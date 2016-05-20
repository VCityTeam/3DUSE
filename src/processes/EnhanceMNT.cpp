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
* @brief A partir d'un ensemble de lignes definissant des routes, cree des polygones en lisant leur largeur
* @param OGRLayer* Layer Contient les donnees du fichier CityGML MNT
*/
OGRGeometry* BuildRoads(OGRLayer* Layer)
{
	OGRGeometry* ListRoads;
	OGRMultiPolygon* MP = new OGRMultiPolygon;

	OGRFeature *Feature;
	int cpt = 0;

	Layer->ResetReading();
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
* @brief A partir d'un MNT Citygml et d'une shapefile de routes lineaires, cree un MNT type avec des polygones TIN et de routes.
* @param MNT : Contient les donnees du fichier CityGML MNT
* @param Roads : Contient les routes du shapefile
* @param MNT_roads : Contiendra le CityModel de sortie avec les routes
* @param TexturesList_roads : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie pour MNT_roads
* @param MNT_ground : Contiendra le CityModel de sortie avec les autres polygones du terrain
* @param TexturesList_ground : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie pour MNT_ground
*/
void CreateRoadsOnMNT(vcity::Tile* MNT, OGRDataSource* Roads, citygml::CityModel* MNT_roads, std::vector<TextureCityGML*>* TexturesList_roads, citygml::CityModel* MNT_ground, std::vector<TextureCityGML*>* TexturesList_ground)
{
	///// TODO : ne pas prendre en comptes les tunnels !!!! /////

	OGRLayer* Layer = Roads->GetLayer(0);

	OGRGeometry* ListRoads = BuildRoads(Layer);

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

					if(PolygonCityGML->getTexture()->getType() == "GeoreferencedTexture") //Ce sont des coordonnees georeferences qu'il faut convertir en coordonnees de texture standard
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


						//////////////////////////////// MARCHE POUR DES TEXTURES 4096x4096 avec un D negatif (donnees de LYON)
						int i = 0;
						for(TVec2f UV:TexUV)
						{
							UV.x = UV.x/4095; 
							UV.y = 1 + UV.y/4095;//Car D est negatif
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

							//Creation des polygones de route

							if(Intersection->getGeometryType() == wkbPolygon || Intersection->getGeometryType() == wkbPolygon25D)
							{
								OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Intersection, &TexUV, &TexUVout); //Pour calculer les coordonnees z et de textures

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

										bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
										for(TextureCityGML* Tex: *TexturesList_roads)
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
											TexturesList_roads->push_back(Texture);
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
										OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Intersection_GC->getGeometryRef(i), &TexUV, &TexUVout); //Pour calculer les coordonnees z et de textures
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

												bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
												for(TextureCityGML* Tex: *TexturesList_roads)
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
													TexturesList_roads->push_back(Texture);
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

													bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
													for(TextureCityGML* Tex: *TexturesList_roads)
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
														TexturesList_roads->push_back(Texture);
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

							//Creation des polygones de MNT (ou Water)

							if(Difference->getGeometryType() == wkbPolygon || Difference->getGeometryType() == wkbPolygon25D)
							{
								OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Difference, &TexUV, &TexUVout); //Pour calculer les coordonnees z et de textures

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

										bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
										for(TextureCityGML* Tex: *TexturesList_ground)
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
											TexturesList_ground->push_back(Texture);
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
										OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Difference_GC->getGeometryRef(i), &TexUV, &TexUVout); //Pour calculer les coordonnees z et de textures
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

												bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
												for(TextureCityGML* Tex: *TexturesList_ground)
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
													TexturesList_ground->push_back(Texture);
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

													bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
													for(TextureCityGML* Tex: *TexturesList_ground)
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
														TexturesList_ground->push_back(Texture);
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

								bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
								for(TextureCityGML* Tex: *TexturesList_ground)
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
									TexturesList_ground->push_back(Texture);
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
				MNT_ground->addCityObject(TIN_CO);
				MNT_ground->addCityObjectAsRoot(TIN_CO);
			}
		}
	}

	++cpt1;

	std::cout << "Avancement : " << cpt1 << " / " << Model->getCityObjectsRoots().size() << std::endl;

	if(Roads_Geo->getPolygons().size() > 0)
	{
		Roads_CO->addGeometry(Roads_Geo);
		MNT_roads->addCityObject(Roads_CO);
		MNT_roads->addCityObjectAsRoot(Roads_CO);
	}

	delete ListRoads;
}


/**
* @brief A partir d'un MNT Citygml et d'une shapefile d'espaces boises, cree un MNT type avec des polygones TIN et de vegetation.
* @param MNT : Contient les donnees du fichier CityGML MNT
* @param Vegetation : Contient les polygones des zones de vegetation
* @param MNT_vegetation : Contiendra le CityModel de sortie avec la vegetation
* @param TexturesList_vegetation : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie pour MNT_vegetation
* @param MNT_ground : Contiendra le CityModel de sortie avec les autres polygones du terrain
* @param TexturesList_ground : La fonction va remplir ce vector avec tous les appels de texture qu'il faudra enregistrer dans le CityGML en sortie pour MNT_ground
*/
void CreateVegetationOnMNT(vcity::Tile* MNT, OGRDataSource* Vegetation, citygml::CityModel* MNT_vegetation, std::vector<TextureCityGML*>* TexturesList_vegetation, citygml::CityModel* MNT_ground, std::vector<TextureCityGML*>* TexturesList_ground)
{
	OGRLayer* Layer = Vegetation->GetLayer(0);

	OGRGeometry* VegetationPolygons;
	OGRMultiPolygon* MP = new OGRMultiPolygon;

	OGRFeature *Feature;
	Layer->ResetReading();

	while((Feature = Layer->GetNextFeature()) != NULL)
	{
		OGRGeometry* Geometry = Feature->GetGeometryRef();

		if(Geometry->getGeometryType() == wkbPolygon || Geometry->getGeometryType() == wkbPolygon25D)
			MP->addGeometry(Geometry);
		else if(Geometry->getGeometryType() == wkbMultiPolygon || Geometry->getGeometryType() == wkbMultiPolygon25D)
		{
			for(int i = 0; i < ((OGRMultiPolygon*)Geometry)->getNumGeometries(); ++i)
				MP->addGeometry(((OGRMultiPolygon*)Geometry)->getGeometryRef(i));
		}
	}

	VegetationPolygons = MP->UnionCascaded();
	delete MP;
	
	citygml::CityModel* Model = MNT->getCityModel();

	citygml::CityObject* Vegetation_CO = new citygml::PlantCover("Vegetation");
	citygml::Geometry* Vegetation_Geo = new citygml::Geometry("VegetationGeometry", citygml::GT_Unknown, 2);

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

					if(PolygonCityGML->getTexture()->getType() == "GeoreferencedTexture") //Ce sont des coordonnees georeferences qu'il faut convertir en coordonnees de texture standard
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


						//////////////////////////////// MARCHE POUR DES TEXTURES 4096x4096 avec un D negatif (donnees de LYON)
						int i = 0;
						for(TVec2f UV:TexUV)
						{
							UV.x = UV.x/4095; 
							UV.y = 1 + UV.y/4095;//Car D est negatif
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
						if(OgrPoly->Intersects(VegetationPolygons))
						{
							OGRGeometry* Intersection = OgrPoly->Intersection(VegetationPolygons);
							OGRGeometry* Difference = OgrPoly->Difference(VegetationPolygons);

							//OGRGeometry* Inter;
							//OGRGeometry* Diff;

							//double A1 = OgrPoly->get_Area();
							//double A2 = 0;
							//double A3 = 0;

							//Creation des polygones de route

							if(Intersection->getGeometryType() == wkbPolygon || Intersection->getGeometryType() == wkbPolygon25D)
							{
								OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Intersection, &TexUV, &TexUVout); //Pour calculer les coordonnees z et de textures

								//Inter = CutPoly->clone();

								if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
								{
									//A2 += ((OGRPolygon*)CutPoly)->get_Area();

									citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_" + std::to_string(cptPolyTIN));
									Vegetation_Geo->addPolygon(GMLPoly);

									if(HasTexture)
									{
										TexturePolygonCityGML Poly;

										Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
										Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
										Poly.TexUV = TexUVout.at(0);

										bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
										for(TextureCityGML* Tex: *TexturesList_vegetation)
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
											TexturesList_vegetation->push_back(Texture);
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
										OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Intersection_GC->getGeometryRef(i), &TexUV, &TexUVout); //Pour calculer les coordonnees z et de textures
										if(CutPoly->getGeometryType() == wkbPolygon || CutPoly->getGeometryType() == wkbPolygon25D)
										{
											//MP->addGeometry(CutPoly);//
											//A2 += ((OGRPolygon*)CutPoly)->get_Area();

											citygml::Polygon* GMLPoly = ConvertOGRPolytoGMLPoly((OGRPolygon*)CutPoly, Name + "_" + std::to_string(cptPolyTIN));
											Vegetation_Geo->addPolygon(GMLPoly);

											if(HasTexture)
											{
												TexturePolygonCityGML Poly;

												Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
												Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
												Poly.TexUV = TexUVout.at(0);

												bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
												for(TextureCityGML* Tex: *TexturesList_vegetation)
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
													TexturesList_vegetation->push_back(Texture);
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
												Vegetation_Geo->addPolygon(GMLPoly);

												if(HasTexture)
												{
													TexturePolygonCityGML Poly;

													Poly.Id = Name + "_" + std::to_string(cptPolyTIN) + "_Poly";
													Poly.IdRing = Name + "_" + std::to_string(cptPolyTIN) + "_Ring";
													Poly.TexUV = TexUVout.at(0);

													bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
													for(TextureCityGML* Tex: *TexturesList_vegetation)
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
														TexturesList_vegetation->push_back(Texture);
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

							//Creation des polygones de MNT (ou Water)

							if(Difference->getGeometryType() == wkbPolygon || Difference->getGeometryType() == wkbPolygon25D)
							{
								OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Difference, &TexUV, &TexUVout); //Pour calculer les coordonnees z et de textures

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

										bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
										for(TextureCityGML* Tex: *TexturesList_ground)
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
											TexturesList_ground->push_back(Texture);
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
										OGRGeometry * CutPoly = CutPolyGMLwithShape(OgrPoly, (OGRPolygon*)Difference_GC->getGeometryRef(i), &TexUV, &TexUVout); //Pour calculer les coordonnees z et de textures
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

												bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
												for(TextureCityGML* Tex: *TexturesList_ground)
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
													TexturesList_ground->push_back(Texture);
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

													bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
													for(TextureCityGML* Tex: *TexturesList_ground)
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
														TexturesList_ground->push_back(Texture);
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

								bool URLTest = false;//Permet de dire si l'URL existe deja dans TexturesList ou non. Si elle n'existe pas, il faut creer un nouveau TextureCityGML pour la stocker.
								for(TextureCityGML* Tex: *TexturesList_ground)
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
									TexturesList_ground->push_back(Texture);
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
				MNT_ground->addCityObject(TIN_CO);
				MNT_ground->addCityObjectAsRoot(TIN_CO);
			}
		}
	}

	++cpt1;

	std::cout << "Avancement : " << cpt1 << " / " << Model->getCityObjectsRoots().size() << std::endl;

	if(Vegetation_Geo->getPolygons().size() > 0)
	{
		Vegetation_CO->addGeometry(Vegetation_Geo);
		MNT_vegetation->addCityObject(Vegetation_CO);
		MNT_vegetation->addCityObjectAsRoot(Vegetation_CO);
	}

	delete VegetationPolygons;
}
