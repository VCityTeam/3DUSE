#include "EnhanceMNT.hpp"
#include "ExportToShape.hpp"
#include "ToolAlgoCut.hpp"
#include <iomanip>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
/**
* @brief A partir d'un ensemble de lignes définissant des routes, crée des polygones en lisant leur largeur
* @param OGRLayer* Layer Contient les données du fichier CityGML MNT
*/
OGRGeometry* BuildRoads(OGRLayer* Layer)
{
	OGRGeometry* ListRoads;
	OGRMultiPolygon* MP = new OGRMultiPolygon;

	OGRFeature *Feature;
	int cpt = 0;
	while((Feature = Layer->GetNextFeature()) != NULL)
	{
		OGRGeometry* Road = Feature->GetGeometryRef();

		if(Road->getGeometryType() == wkbLineString || Road->getGeometryType() == wkbLineString25D)
		{
			double L = 2;
			if(Feature->GetFieldIndex("LARGEUR") != -1)
				L = Feature->GetFieldAsDouble("LARGEUR");

			if(L == 0)
				L = 2;

			OGRGeometry* RoadPoly = Road->Buffer(L);

			if(RoadPoly->getGeometryType() == wkbPolygon || RoadPoly->getGeometryType() == wkbPolygon25D)
			{
				MP->addGeometry(RoadPoly);
			}
			else
				std::cout << "Erreur dilatation : " << RoadPoly->getGeometryName() << std::endl;
		}
		else
		{
			std::cout << "Route non LineString : " << Road->getGeometryName() << std::endl;
		}
		++cpt;
	}

	//SaveGeometrytoShape("Roads.shp", MP);

	ListRoads = MP->UnionCascaded();

	//SaveGeometrytoShape("RoadsUnion.shp", ListRoads);

	delete MP;

	return ListRoads;
}

////////////////////////////////////////////////////////////////////////////////
/**
* @brief A partir d'un MNT Citygml et d'une shapefile de routes linéaires, crée un MNT typé avec des polygones TIN et de routes.
* @param MNT Contient les données du fichier CityGML MNT
* @param Roads Contient les routes du shapefile
* @param TexturesList : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie
*/
citygml::CityModel* CreateRoadsOnMNT(vcity::Tile* MNT, OGRDataSource* Roads, std::vector<TextureCityGML*>* TexturesList)
{
	OGRLayer* Layer = Roads->GetLayer(0);

	OGRGeometry* ListRoads = BuildRoads(Layer);

	citygml::CityModel* MNT_Type = new citygml::CityModel;
	citygml::CityModel* Model = MNT->getCityModel();

	citygml::CityObject* Roads_CO = new citygml::Road("Road");
	citygml::Geometry* Roads_Geo = new citygml::Geometry("RoadGeometry", citygml::GT_Unknown, 2);

	int cpt1 = -1;

	for(citygml::CityObject* obj : Model->getCityObjectsRoots())
	{
		++cpt1;

		std::cout << "Avancement : " << cpt1 << " / " << Model->getCityObjectsRoots().size() << std::endl;
		if(obj->getType() == citygml::COT_TINRelief || obj->getType() == citygml::COT_WaterBody)
		{
			std::string Name = obj->getId();
			citygml::CityObject* TIN_CO;
			if(obj->getType() == citygml::COT_TINRelief)
				TIN_CO = new citygml::TINRelief(Name);
			else if(obj->getType() == citygml::COT_WaterBody)
				TIN_CO = new citygml::WaterBody(Name);

			citygml::Geometry* TIN_Geo = new citygml::Geometry(Name + "_TINGeometry", citygml::GT_Unknown, 2);

			int cptPolyTIN = 0;

			for(citygml::Geometry* Geometry : obj->getGeometries())
			{
				for(citygml::Polygon * PolygonCityGML : Geometry->getPolygons())
				{
					bool HasTexture = (PolygonCityGML->getTexture() != nullptr);

					std::string Url;
					citygml::Texture::WrapMode WrapMode;
					std::vector<std::vector<TVec2f>> TexUVout;
					if(HasTexture)
					{
						Url = PolygonCityGML->getTexture()->getUrl();
						WrapMode = PolygonCityGML->getTexture()->getWrapMode();
					}

					std::vector<TVec2f> TexUV = PolygonCityGML->getTexCoords();

					OGRLinearRing * OgrRing = new OGRLinearRing;
					for(TVec3d Point : PolygonCityGML->getExteriorRing()->getVertices())
						OgrRing->addPoint(Point.x, Point.y, Point.z);

					if(PolygonCityGML->getTexture()->getType() == "GeoreferencedTexture") //Ce sont des coordonnées géoréférences qu'il faut convertir en coordonnées de texture standard
					{
						/*double A, B, C ,D; //Voir fr.wikipedia.org/wiki/World_file : Taille pixel, rotation, retournement //Pour faire une conversion propre.
						double offset_x;
						double offset_y;

						std::string path = PathFolder + "/" + PolygonCityGML->getTexture()->getUrl().substr(0, PolygonCityGML->getTexture()->getUrl().find_last_of('.'))+".jgw";
						std::cout << path << std::endl;
						std::ifstream fichier(path, std::ios::in);

						if(fichier)
						{
						fichier >> A >> B >> C >> D >> offset_x >> offset_y;
						fichier.close();
						}
						std::cout << A << " " << B << " " << C << " " << D << " " << offset_x << " " << offset_y << std::endl;*/


						//////////////////////////////// MARCHE POUR DES TEXTURES 4096x4096 avec un D négatif (données de LYON)
						int i = 0;
						for(TVec2f UV:TexUV)
						{
							UV.x = UV.x/4095; 
							UV.y = 1 + UV.y/4095;//Car D est négatif
							TexUV.at(i) = UV;
							++i;
						}
					}

					OgrRing->closeRings();
					if(OgrRing->getNumPoints() > 3)
					{
						OGRPolygon * OgrPoly = new OGRPolygon;
						OgrPoly->addRingDirectly(OgrRing);
						if(!OgrPoly->IsValid())
							continue;
						if(OgrPoly->Intersects(ListRoads))
						{
							OGRGeometry* Intersection = OgrPoly->Intersection(ListRoads);
							OGRGeometry* Difference = OgrPoly->Difference(ListRoads);

							//OGRGeometry* Inter;
							//OGRGeometry* Diff;

							//double A1 = OgrPoly->get_Area();
							//double A2 = 0;
							//double A3 = 0;

							//Création des polygones de route

							if(Intersection->getGeometryType() == wkbPolygon || Intersection->getGeometryType() == wkbPolygon25D)
							{
								OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Intersection, &TexUV, &TexUVout); //Pour calculer les coordonnées z et de textures

								//Inter = CutPoly->clone();

								if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
								{
									//A2 += ((OGRPolygon*)CutPoly)->get_Area();

									citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_" + std::to_string(cptPolyTIN));
									Roads_Geo->addPolygon(GMLPoly);

									if(HasTexture)
									{
										TexturePolygonCityGML Poly;

										Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
										Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
										Poly.TexUV = TexUVout.at(0);

										bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
										for(TextureCityGML* Tex: *TexturesList)
										{
											if(Tex->Url == Url)
											{
												URLTest = true;
												Tex->ListPolygons.push_back(Poly);
												break;
											}
										}
										if(!URLTest)
										{
											TextureCityGML* Texture = new TextureCityGML;
											Texture->Wrap = WrapMode;
											Texture->Url = Url;
											Texture->ListPolygons.push_back(Poly);
											TexturesList->push_back(Texture);
										}
									}
									++cptPolyTIN;
								}
								else
									std::cout << "ERREUR : INTER NON POLYGON" << std::endl;
								delete CutPoly;
							}
							else if(Intersection->getGeometryType() == wkbGeometryCollection || Intersection->getGeometryType() == wkbGeometryCollection25D || Intersection->getGeometryType() == wkbMultiPolygon || Intersection->getGeometryType() == wkbMultiPolygon25D) //Dans ce cas,il faut aller chercher les polygones un par un pour les ajouter.
							{
								//OGRMultiPolygon* MP =  new OGRMultiPolygon;//

								OGRGeometryCollection* Intersection_GC = (OGRGeometryCollection*)Intersection;
								for(int i = 0; i < Intersection_GC->getNumGeometries(); ++i)
								{
									if(Intersection_GC->getGeometryRef(i)->getGeometryType() == wkbPolygon || Intersection_GC->getGeometryRef(i)->getGeometryType() == wkbPolygon25D)
									{
										OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Intersection_GC->getGeometryRef(i), &TexUV, &TexUVout); //Pour calculer les coordonnées z et de textures
										if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
										{
											//MP->addGeometry(CutPoly);//
											//A2 += ((OGRPolygon*)CutPoly)->get_Area();

											citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_" + std::to_string(cptPolyTIN));
											Roads_Geo->addPolygon(GMLPoly);

											if(HasTexture)
											{
												TexturePolygonCityGML Poly;

												Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
												Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
												Poly.TexUV = TexUVout.at(0);

												bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
												for(TextureCityGML* Tex: *TexturesList)
												{
													if(Tex->Url == Url)
													{
														URLTest = true;
														Tex->ListPolygons.push_back(Poly);
														break;
													}
												}
												if(!URLTest)
												{
													TextureCityGML* Texture = new TextureCityGML;
													Texture->Wrap = WrapMode;
													Texture->Url = Url;
													Texture->ListPolygons.push_back(Poly);
													TexturesList->push_back(Texture);
												}
											}
											++cptPolyTIN;
										}
										else if(CutPoly->getGeometryType() == wkbMultiPolygon || CutPoly->getGeometryType() == wkbMultiPolygon25D)
										{
											OGRMultiPolygon* CutPoly_MP = (OGRMultiPolygon*) CutPoly;
											for(int j = 0; j < CutPoly_MP->getNumGeometries(); ++j)
											{
												//MP->addGeometry(CutPoly_MP->getGeometryRef(j));//
												//A2 = ((OGRPolygon*)(OGRPolygon*)CutPoly_MP->getGeometryRef(j))->get_Area();

												citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly_MP->getGeometryRef(j), Name + "_" + std::to_string(cptPolyTIN));
												Roads_Geo->addPolygon(GMLPoly);

												if(HasTexture)
												{
													TexturePolygonCityGML Poly;

													Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
													Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
													Poly.TexUV = TexUVout.at(0);

													bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
													for(TextureCityGML* Tex: *TexturesList)
													{
														if(Tex->Url == Url)
														{
															URLTest = true;
															Tex->ListPolygons.push_back(Poly);
															break;
														}
													}
													if(!URLTest)
													{
														TextureCityGML* Texture = new TextureCityGML;
														Texture->Wrap = WrapMode;
														Texture->Url = Url;
														Texture->ListPolygons.push_back(Poly);
														TexturesList->push_back(Texture);
													}
												}
												++cptPolyTIN;
											}
										}
										delete CutPoly;
									}
								}

								//Inter = MP;
							}

							//Création des polygones de MNT (ou Water)

							if(Difference->getGeometryType() == wkbPolygon || Difference->getGeometryType() == wkbPolygon25D)
							{
								OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Difference, &TexUV, &TexUVout); //Pour calculer les coordonnées z et de textures

								//Diff = CutPoly->clone();

								if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
								{
									//A3 += ((OGRPolygon*)CutPoly)->get_Area();

									citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_" + std::to_string(cptPolyTIN));
									TIN_Geo->addPolygon(GMLPoly);

									if(HasTexture)
									{
										TexturePolygonCityGML Poly;

										Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
										Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
										Poly.TexUV = TexUVout.at(0);

										bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
										for(TextureCityGML* Tex: *TexturesList)
										{
											if(Tex->Url == Url)
											{
												URLTest = true;
												Tex->ListPolygons.push_back(Poly);
												break;
											}
										}
										if(!URLTest)
										{
											TextureCityGML* Texture = new TextureCityGML;
											Texture->Wrap = WrapMode;
											Texture->Url = Url;
											Texture->ListPolygons.push_back(Poly);
											TexturesList->push_back(Texture);
										}
									}
									++cptPolyTIN;
								}
								else
									std::cout << "ERREUR : INTER NON POLYGON" << std::endl;
								delete CutPoly;
							}
							else if(Difference->getGeometryType() == wkbGeometryCollection || Difference->getGeometryType() == wkbGeometryCollection25D || Difference->getGeometryType() == wkbMultiPolygon || Difference->getGeometryType() == wkbMultiPolygon25D) //Dans ce cas,il faut aller chercher les polygones un par un pour les ajouter.
							{
								//OGRMultiPolygon* MP =  new OGRMultiPolygon;//

								OGRGeometryCollection* Difference_GC = (OGRGeometryCollection*)Difference;
								for(int i = 0; i < Difference_GC->getNumGeometries(); ++i)
								{
									if(Difference_GC->getGeometryRef(i)->getGeometryType() == wkbPolygon || Difference_GC->getGeometryRef(i)->getGeometryType() == wkbPolygon25D)
									{
										OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Difference_GC->getGeometryRef(i), &TexUV, &TexUVout); //Pour calculer les coordonnées z et de textures
										if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
										{
											//MP->addGeometry(CutPoly);//
											//A3 += ((OGRPolygon*)CutPoly)->get_Area();

											citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_" + std::to_string(cptPolyTIN));
											TIN_Geo->addPolygon(GMLPoly);

											if(HasTexture)
											{
												TexturePolygonCityGML Poly;

												Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
												Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
												Poly.TexUV = TexUVout.at(0);

												bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
												for(TextureCityGML* Tex: *TexturesList)
												{
													if(Tex->Url == Url)
													{
														URLTest = true;
														Tex->ListPolygons.push_back(Poly);
														break;
													}
												}
												if(!URLTest)
												{
													TextureCityGML* Texture = new TextureCityGML;
													Texture->Wrap = WrapMode;
													Texture->Url = Url;
													Texture->ListPolygons.push_back(Poly);
													TexturesList->push_back(Texture);
												}
											}
											++cptPolyTIN;
										}
										else if(CutPoly->getGeometryType() == wkbMultiPolygon || CutPoly->getGeometryType() == wkbMultiPolygon25D)
										{
											OGRMultiPolygon* CutPoly_MP = (OGRMultiPolygon*) CutPoly;
											for(int j = 0; j < CutPoly_MP->getNumGeometries(); ++j)
											{
												//MP->addGeometry(CutPoly_MP->getGeometryRef(j)); //
												//A3 += ((OGRPolygon*)CutPoly_MP->getGeometryRef(j))->get_Area();

												citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly_MP->getGeometryRef(j), Name + "_" + std::to_string(cptPolyTIN));
												TIN_Geo->addPolygon(GMLPoly);

												if(HasTexture)
												{
													TexturePolygonCityGML Poly;

													Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
													Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
													Poly.TexUV = TexUVout.at(0);

													bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
													for(TextureCityGML* Tex: *TexturesList)
													{
														if(Tex->Url == Url)
														{
															URLTest = true;
															Tex->ListPolygons.push_back(Poly);
															break;
														}
													}
													if(!URLTest)
													{
														TextureCityGML* Texture = new TextureCityGML;
														Texture->Wrap = WrapMode;
														Texture->Url = Url;
														Texture->ListPolygons.push_back(Poly);
														TexturesList->push_back(Texture);
													}
												}
												++cptPolyTIN;
											}
										}
										delete CutPoly;
									}
								}

								//Diff = MP;
							}


							/*if(abs(A2 + A3 - A1) > 0.1)
							{
								std::cout << A1 << std::endl;
								std::cout << A2 << " + " << A3 << " = " << A2 + A3 << std::endl;

								SaveGeometrytoShape("A_MNTPoly.shp", OgrPoly);
								std::cout << "Intersection : " << Intersection->getGeometryName() << std::endl;
								SaveGeometrytoShape("B_Inter.shp", Intersection);
								std::cout << "Difference : " << Difference->getGeometryName() << std::endl;
								SaveGeometrytoShape("B_Diff.shp", Difference);
								std::cout << "Inter : " << Inter->getGeometryName() << std::endl;
								SaveGeometrytoShape("A_Inter.shp", Inter);
								std::cout << "Diff : " << Diff->getGeometryName() << std::endl;
								SaveGeometrytoShape("A_Diff.shp", Diff);
								int a;
								std::cin >> a;
							}
							delete Inter;
							delete Diff;*/
						}
						else //Si le polygone de MNT n'intersecte pas les routes, alors on le remet tel quel.
						{
							TIN_Geo->addPolygon(PolygonCityGML);
							if(HasTexture)
							{
								TexturePolygonCityGML Poly;

								Poly.Id = PolygonCityGML->getId();
								Poly.IdRing = PolygonCityGML->getExteriorRing()->getId();
								Poly.TexUV = TexUV;

								bool URLTest = false;//Permet de dire si l'URL existe déjà dans TexturesList ou non. Si elle n'existe pas, il faut créer un nouveau TextureCityGML pour la stocker.
								for(TextureCityGML* Tex: *TexturesList)
								{
									if(Tex->Url == Url)
									{
										URLTest = true;
										Tex->ListPolygons.push_back(Poly);
										break;
									}
								}
								if(!URLTest)
								{
									TextureCityGML* Texture = new TextureCityGML;
									Texture->Wrap = WrapMode;
									Texture->Url = Url;
									Texture->ListPolygons.push_back(Poly);
									TexturesList->push_back(Texture);
								}
							}
						}
						delete OgrPoly;
					}
					else
						delete OgrRing;
				}
			}

			if(TIN_Geo->getPolygons().size() > 0)
			{
				TIN_CO->addGeometry(TIN_Geo);
				MNT_Type->addCityObject(TIN_CO);
				MNT_Type->addCityObjectAsRoot(TIN_CO);
			}
		}
	}

	if(Roads_Geo->getPolygons().size() > 0)
	{
		Roads_CO->addGeometry(Roads_Geo);
		MNT_Type->addCityObject(Roads_CO);
		MNT_Type->addCityObjectAsRoot(Roads_CO);
	}

	return MNT_Type;
}
