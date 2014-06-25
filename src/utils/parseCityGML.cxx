#include <stdio.h>
#include <stdlib.h>
#include <libxml/tree.h>
#include <libxml/parser.h>

#include <string>
#include <set>

#include <iostream>
#include <QFileInfo>
#include <QDir>
#include <QFile>

#include "vecs.hpp"

#define TEXTURE_PROCESS			1

#define MAX_POINTS_IN_POSLIST	200	// TEMP

struct FOUR_PLANES
{
	TVec3d n[4];
	TVec3d p0[4];
};

// TEMP
FOUR_PLANES G_my4planes;
double G_xmin, G_ymin, G_xmax, G_ymax;
// TEMP

// todo:
//------
// 1. textures (0->1)
// 2. better xmlUnlinkNode
// 3. split polygons > 3

// ---
// adapted from http://www.scratchapixel.com/lessons/3d-basic-lessons/lesson-7-intersecting-simple-shapes/ray-plane-and-ray-disk-intersection/
bool intersectPlane(const TVec3d &n, const TVec3d &p0, const TVec3d& l0, const TVec3d &l, double &d)
{
	// assuming vectors are all normalized
	double denom = n.dot(l);
	if (fabs(denom) > 1e-6) // fabs add by MM-MT
	{
		TVec3d p0l0 = p0 - l0;
		d = n.dot(p0l0) / denom;
		return (d >= 0);
	}
	return false;
}
// ---

void process_Building_ReliefFeature_boundingbox(xmlNodePtr noeud, bool *first_posList, double *xmin, double *ymin, double *zmin, double *xmax, double *ymax, double *zmax, std::set<std::string> *UUID_s)
{
	if (noeud->type == XML_ELEMENT_NODE)
	{
        xmlChar *chemin = xmlGetNodePath(noeud);
        if (noeud->children != NULL)// && noeud->children->type == XML_TEXT_NODE) // MT
		{
            xmlChar *contenu = xmlNodeGetContent(noeud);
			if (xmlStrEqual(noeud->name, BAD_CAST "posList"))
			{
				//printf("%s -> %s\n", chemin, contenu);
				//printf("posList: %s\n", contenu);

				if (TEXTURE_PROCESS)
				{
					xmlNodePtr noeudPolygon = noeud->parent->parent->parent;
					xmlNodePtr noeudSurface = noeudPolygon->parent->parent;

					//printf("noeudPolygon: %s - noeudSurface: %s\n", noeudPolygon->name, noeudSurface->name);

					if (xmlGetProp(noeudPolygon, BAD_CAST "id"))
					{
						if ( UUID_s->find((char *) xmlGetProp(noeudPolygon, BAD_CAST "id")) == UUID_s->end() )
							UUID_s->insert((char *) xmlGetProp(noeudPolygon, BAD_CAST "id"));

						//printf("id Polygon: %s\n", xmlGetProp(noeudPolygon, BAD_CAST "id"));
					}
					else
					{
						if ( UUID_s->find((char *) xmlGetProp(noeudSurface, BAD_CAST "id")) == UUID_s->end() )
							UUID_s->insert((char *) xmlGetProp(noeudSurface, BAD_CAST "id"));

						//printf("id Surface: %s\n", xmlGetProp(noeudSurface, BAD_CAST "id"));
					}
				}

				// init
				char *endptr = NULL;
				bool first = true;

				double x, y, z;

				double xmin_posList, ymin_posList, zmin_posList;
				double xmax_posList, ymax_posList, zmax_posList;
				xmin_posList = ymin_posList = zmin_posList = xmax_posList = ymax_posList = zmax_posList = 0.;

				do
				{
					if (!endptr) endptr = (char *) contenu;

					x = strtod(endptr, &endptr);
					if (x != 0. && first) { xmin_posList = xmax_posList = x; } if (x != 0. && !first && x < xmin_posList) { xmin_posList = x; } if (x != 0. && !first && x > xmax_posList) { xmax_posList = x; }
					y = strtod(endptr, &endptr);
					if (y != 0. && first) { ymin_posList = ymax_posList = y; } if (y != 0. && !first && y < ymin_posList) { ymin_posList = y; } if (y != 0. && !first && y > ymax_posList) { ymax_posList = y; }
					z = strtod(endptr, &endptr);
					if (z != 0. && first) { zmin_posList = zmax_posList = z; } if (z != 0. && !first && z < zmin_posList) { zmin_posList = z; } if (z != 0. && !first && z > zmax_posList) { zmax_posList = z; }
					//printf("(%lf %lf %lf)\n", x, y, z);
					first = false;
				}
				while ( !( (x == 0.) && (y == 0.) && (z == 0.) ) );

				//printf("MIN_posList: (%lf %lf %lf)\n", xmin_posList, ymin_posList, zmin_posList);
				//printf("MAX_posList: (%lf %lf %lf)\n", xmax_posList, ymax_posList, zmax_posList);

				if (xmin_posList != 0. && (*first_posList)) { *xmin = xmin_posList; } if (xmin_posList != 0. && !(*first_posList) && xmin_posList < *xmin) { *xmin = xmin_posList; }
				if (ymin_posList != 0. && (*first_posList)) { *ymin = ymin_posList; } if (ymin_posList != 0. && !(*first_posList) && ymin_posList < *ymin) { *ymin = ymin_posList; }
				if (zmin_posList != 0. && (*first_posList)) { *zmin = zmin_posList; } if (zmin_posList != 0. && !(*first_posList) && zmin_posList < *zmin) { *zmin = zmin_posList; }
				if (xmax_posList != 0. && (*first_posList)) { *xmax = xmax_posList; } if (xmax_posList != 0. && !(*first_posList) && xmax_posList > *xmax) { *xmax = xmax_posList; }
				if (ymax_posList != 0. && (*first_posList)) { *ymax = ymax_posList; } if (ymax_posList != 0. && !(*first_posList) && ymax_posList > *ymax) { *ymax = ymax_posList; }
				if (zmax_posList != 0. && (*first_posList)) { *zmax = zmax_posList; } if (zmax_posList != 0. && !(*first_posList) && zmax_posList > *zmax) { *zmax = zmax_posList; }
				*first_posList = false;

				// ---
				if ( !(*xmax < G_xmin) && !(*ymax < G_ymin) && !(*xmin > G_xmax) && !(*ymin > G_ymax) )
				{
					// init
					endptr = NULL;
					first = true;

					TVec3d l0[MAX_POINTS_IN_POSLIST+1];		// TEMP
					TVec3d l1[MAX_POINTS_IN_POSLIST+1+1];	// TEMP
					TVec3d l[MAX_POINTS_IN_POSLIST];		// TEMP
					int i;

					do
					{
						if (!endptr) endptr = (char *) contenu;
						if (first)
							i=0;
						else
							i++;

						if (i > (MAX_POINTS_IN_POSLIST+1))	// TEMP
						{
							printf("---> STOP ---> PLEASE, INCREASE MAX_POINTS_IN_POSLIST IN SOURCE CODE\n");
							exit(-1);
						}

						l0[i].x = strtod(endptr, &endptr);
						l0[i].y = strtod(endptr, &endptr);
						l0[i].z = strtod(endptr, &endptr);
						//printf("p%d - %lf,%lf,%lf\n", i, l0[i].x, l0[i].y, l0[i].z);
						first = false;
					}
					while ( !( (l0[i].x == 0.) && (l0[i].y == 0.) && (l0[i].z == 0.) ) );
					i--;
					//printf("nb points: %d\n", i);

					for (int s=0; s<i; s++)
					{
						l[s]=l0[s+1]-l0[s];
						//l[s]=l[s].normal(); // normalizing

						//printf("s%d - p1: %lf,%lf,%lf - p2: %lf,%lf,%lf\n", s, l0[s].x, l0[s].y, l0[s].z, l0[s+1].x, l0[s+1].y, l0[s+1].z);
					}

					double d;
					int j=0;
					TVec3d l1_temp;
					bool inter, coin;
					for (int s=0; s<i; s++)
					{
						//printf("segment: %ld\n", s);
						inter=coin=false;

						for (int p=0; p<4; p++)
						{
							//printf("plan: %ld\n", p);

							// intersection
							if ( (intersectPlane(G_my4planes.n[p], G_my4planes.p0[p], l0[s], l[s], d)) && d>0. && d<1. )
							{
								//printf("INTER s%d-p%d - %lf\n", s, p, d);

								if ( (l0[s+1].x >= G_xmin) && (l0[s+1].x <= G_xmax) )
									if ( (l0[s+1].y >= G_ymin) && (l0[s+1].y <= G_ymax) )
									{
										//printf(" -> KEEP INTER s+1\n");

										l1_temp = l0[s]+l[s]*d;
										if ( (j==0) || (l1[j-1] != l1_temp) )
										{
											l1[j] = l1_temp; j++;
											inter=true;
										}
										l1_temp = l0[s+1];
										if ( (j==0) || (l1[j-1] != l1_temp) )
										{
											l1[j] = l1_temp; j++;
											inter=true;
										}
									}

								if ( (l0[s].x >= G_xmin) && (l0[s].x <= G_xmax) )
									if ( (l0[s].y >= G_ymin) && (l0[s].y <= G_ymax) )
									{
										//printf(" -> KEEP INTER s\n");

										l1_temp = l0[s];
										if ( (j==0) || (l1[j-1] != l1_temp) )
										{
											l1[j] = l1_temp; j++;
											inter=true;
										}
										l1_temp = l0[s]+l[s]*d;
										if ( (j==0) || (l1[j-1] != l1_temp) )
										{
											l1[j] = l1_temp; j++;
											inter=true;
										}
									}	
							}
						}

						// gestion des 4 coins
						if (!inter)
						{
							// new c0
							if ( (l0[s].x < G_xmin) && (l0[s+1].y < G_ymin) )
							{
								l1_temp.x = G_xmin; l1_temp.y = G_ymin; l1_temp.z = (l0[s].z + l0[s+1].z)/2.; // TODO BETTER Z
								if ( (j==0) || (l1[j-1] != l1_temp) )
								{
									l1[j] = l1_temp; j++;
									coin=true;
								}
							}
							// new c3
							else if ( (l0[s+1].x < G_xmin) && (l0[s].y > G_ymax) )
							{
								l1_temp.x = G_xmin; l1_temp.y = G_ymax; l1_temp.z = (l0[s].z + l0[s+1].z)/2.; // TODO BETTER Z
								if ( (j==0) || (l1[j-1] != l1_temp) )
								{
									l1[j] = l1_temp; j++;
									coin=true;
								}
							}
							// new c1
							else if ( (l0[s+1].x > G_xmax) && (l0[s].y < G_ymin) )
							{
								l1_temp.x = G_xmax; l1_temp.y = G_ymin; l1_temp.z = (l0[s+1].z + l0[s].z)/2.; // TODO BETTER Z
								if ( (j==0) || (l1[j-1] != l1_temp) )
								{
									l1[j] = l1_temp; j++;
									coin=true;
								}
							}
							// new c2
							else if ( (l0[s].x > G_xmax) && (l0[s+1].y > G_ymax) )
							{
								l1_temp.x = G_xmax; l1_temp.y = G_ymax; l1_temp.z = (l0[s+1].z + l0[s].z)/2.; // TODO BETTER Z
								if ( (j==0) || (l1[j-1] != l1_temp) )
								{
									l1[j] = l1_temp; j++;
									coin=true;
								}
							}
						}

						if ( (!inter) && (!coin) )
						{
							if ( (j==0) || (l1[j-1] != l0[s]) )
							{
								l1[j] = l0[s]; j++;
							}
						}
					}

					std::string new_posList = "";
					int new_nb_points = 0;
					bool first_point=true; int first_p, last_p;
					for (int p=0; p<j; p++)
					{
						if ( (l1[p].x >= G_xmin) && (l1[p].x <= G_xmax) )
							if ( (l1[p].y >= G_ymin) && (l1[p].y <= G_ymax) )
							{
								new_posList += std::to_string(l1[p].x); new_posList += " ";
								new_posList += std::to_string(l1[p].y); new_posList += " ";
								new_posList += std::to_string(l1[p].z); new_posList += " ";

								if (first_point)
								{
									first_point = false;
									first_p=p;
								}
								last_p=p;

								new_nb_points++;
							}
					}

					if ( (l1[first_p].x == l1[last_p].x) && (l1[first_p].y == l1[last_p].y) && (l1[first_p].z == l1[last_p].z) )
					{
						//std::cout << "new posList (nb points: " << new_nb_points << "): " << new_posList << std::endl;
					}
					else
					{
						new_posList += std::to_string(l1[first_p].x); new_posList += " ";
						new_posList += std::to_string(l1[first_p].y); new_posList += " ";
						new_posList += std::to_string(l1[first_p].z); new_posList += " ";

						new_nb_points++;
						//std::cout << "ADD ONE POINT: new posList (new nb points: " << new_nb_points << "): " << new_posList << std::endl;
					}
					if (new_nb_points >= 3)
					{
						xmlNodeSetContent(noeud, BAD_CAST new_posList.c_str());
					}
					else
					{
						xmlUnlinkNode(noeud);
						xmlFree(noeud);
					}
				}
				else
				{
					xmlUnlinkNode(noeud);
					xmlFree(noeud);

					//std::cout << "---> PolygonSurfaceOUT" << std::endl;
				}
				// ---
			}
			//printf("\n");
            xmlFree(contenu);
        }
		else
		{
			//printf("%s\n", chemin);
        }
        xmlFree(chemin);
    }
}

typedef void (*fct_process_Building_ReliefFeature_boundingbox)(xmlNodePtr, bool *, double *, double *, double *, double *, double *, double *, std::set<std::string> *);
void parcours_prefixe_Building_ReliefFeature_boundingbox(xmlNodePtr noeud, fct_process_Building_ReliefFeature_boundingbox f, bool *first, double *xmin, double *ymin, double *zmin, double *xmax, double *ymax, double *zmax, std::set<std::string> *UUID_s)
{
    xmlNodePtr n;
    
    for (n = noeud; n != NULL; n = n->next)
	{
        f(n, first, xmin, ymin, zmin, xmax, ymax, zmax, UUID_s);

        if ((n->type == XML_ELEMENT_NODE) && (n->children != NULL))
		{
            parcours_prefixe_Building_ReliefFeature_boundingbox(n->children, f, first, xmin, ymin, zmin, xmax, ymax, zmax, UUID_s);
        }
    }
}

void copy_texture_files(xmlNodePtr noeud, std::string folderIN, std::string folderOUT)
{
	xmlChar *imageURI = xmlNodeGetContent(noeud);

		QString textIN, textOUT;
		textIN = QString::fromStdString(folderIN)+"/"+QString::fromStdString(std::string((char *) imageURI));
		textOUT = QString::fromStdString(folderOUT)+"/"+QString::fromStdString(std::string((char *) imageURI));

		QFileInfo fiOUT(textOUT);
		QDir dir(fiOUT.absolutePath());
		if (!dir.exists())
			dir.mkpath(".");

		//std::cout << " -> textIN: " << textIN.toStdString() << std::endl;
		QFile::copy(textIN, textOUT);
		//printf(" -> texture COPIED: %s\n", (char *) imageURI);
		//std::cout << " -> textOUT: " << textOUT.toStdString() << std::endl;

		QFileInfo fiIN(textIN);
		textIN = fiIN.absolutePath()+"/"+fiIN.baseName()+"."+fiIN.suffix().at(0)+fiIN.suffix().at(2)+"w";
		QFile filew(textIN);
		if (filew.exists())
		{
			textOUT = fiOUT.absolutePath()+"/"+fiOUT.baseName()+"."+fiOUT.suffix().at(0)+fiOUT.suffix().at(2)+"w";
			QFile::copy(textIN, textOUT);					
			//std::cout << " -> textOUTwf: " << textOUT.toStdString() << std::endl;
		}

	xmlFree(imageURI);
}

void process_textures(xmlNodePtr n, xmlNodePtr copy_node3, std::set<std::string> *UUID_s, std::string folderIN, std::string folderOUT)
{
	// CAUTION : if copy_node3 = NULL -> xml nodes not copied, just texture files are copied !
	if ((n->type == XML_ELEMENT_NODE) && (n->children != NULL))
	{
		for (n = n->children; n != NULL; n = n->next)
		{
			if (xmlStrEqual(n->name, BAD_CAST "Appearance"))
			{
				xmlNodePtr copy_node4 = xmlCopyNode(n, 2);
				if (copy_node3)
					xmlAddChild(copy_node3, copy_node4);

				if ((n->type == XML_ELEMENT_NODE) && (n->children != NULL))
				{
					for (xmlNodePtr nc = n->children; nc != NULL; nc = nc->next)
					{
						if (xmlStrEqual(nc->name, BAD_CAST "surfaceDataMember"))
						{
							xmlNodePtr copy_node5 = xmlCopyNode(nc, 2);

							// --- GeoreferencedTexture ---
							if (xmlStrEqual(nc->children->name, BAD_CAST "GeoreferencedTexture"))
							{
								xmlNodePtr copy_node6 = xmlCopyNode(nc->children, 1);
								xmlNodePtr copy_node_imageURI;

								if ((nc->children->type == XML_ELEMENT_NODE) && (nc->children->children != NULL))
								{
									bool node_copied = false;

									for (xmlNodePtr nc2 = nc->children->children; nc2 != NULL; nc2 = nc2->next)
									{
										if (xmlStrEqual(nc2->name, BAD_CAST "target"))
										{
											xmlChar *contenu = xmlNodeGetContent(nc2);
											char *p = (char *) contenu;
											p++;

												if ( UUID_s->find(p) != UUID_s->end() )
												{
													if (copy_node3)
													{
														xmlAddChild(copy_node4, copy_node5);
														xmlAddChild(copy_node5, copy_node6);
															
														printf("GeoreferencedTexture target COPIED: %s\n", p);
													}

													node_copied = true;
												}

											xmlFree(contenu);
										}
										else if (xmlStrEqual(nc2->name, BAD_CAST "imageURI"))
											copy_node_imageURI = nc2;
									}

									if (node_copied)
										copy_texture_files(copy_node_imageURI, folderIN, folderOUT);
								}
							}
							// --- ParameterizedTexture ---
							else if (xmlStrEqual(nc->children->name, BAD_CAST "ParameterizedTexture"))
							{
								xmlNodePtr copy_node6 = xmlCopyNode(nc->children, 2);
								xmlNodePtr copy_node_imageURI, copy_node_textureType, copy_node_wrapMode, copy_node_borderColor;

								if ((nc->children->type == XML_ELEMENT_NODE) && (nc->children->children != NULL))
								{
									bool first_target = true;

									for (xmlNodePtr nc2 = nc->children->children; nc2 != NULL; nc2 = nc2->next)
									{
										if (xmlStrEqual(nc2->name, BAD_CAST "target"))
										{
											xmlNodePtr copy_node_target = xmlCopyNode(nc2, 1);
													
											xmlChar *prop = xmlGetProp(nc2, BAD_CAST "uri");
											char *p = (char *) prop;
											p++;

												if ( UUID_s->find(p) != UUID_s->end() )
												{
													if (first_target)
													{

														if (copy_node3)
														{
															xmlAddChild(copy_node4, copy_node5);
															xmlAddChild(copy_node5, copy_node6);
															xmlAddChild(copy_node6, copy_node_imageURI);
														}
														copy_texture_files(copy_node_imageURI, folderIN, folderOUT);																
														if (copy_node3)
														{
															xmlAddChild(copy_node6, copy_node_textureType);
															xmlAddChild(copy_node6, copy_node_wrapMode);
															xmlAddChild(copy_node6, copy_node_borderColor);
														}

														first_target = false;
													}
													if (copy_node3)
													{
														xmlAddChild(copy_node6, copy_node_target);

														printf("ParameterizedTexture target COPIED: %s\n", p);
													}
												}

											xmlFree(prop); // necessary ???
										}
										else if (xmlStrEqual(nc2->name, BAD_CAST "imageURI"))
											copy_node_imageURI = xmlCopyNode(nc2, 1);
										else if (xmlStrEqual(nc2->name, BAD_CAST "textureType"))
											copy_node_textureType = xmlCopyNode(nc2, 1);
										else if (xmlStrEqual(nc2->name, BAD_CAST "wrapMode"))
											copy_node_wrapMode = xmlCopyNode(nc2, 1);
										else if (xmlStrEqual(nc2->name, BAD_CAST "borderColor"))
											copy_node_borderColor = xmlCopyNode(nc2, 1);
										else
										{
											if (copy_node3)
												printf(" -> CAUTION: ParameterizedTexture child NOT COPIED: %s\n", nc2->name);
										}
									}
								}
							}
						}					
						else
						{
							if (copy_node3)
								printf(" -> NOT COPIED: %s\n", nc->name);
						}
					}
				}
			}					
			else
			{
				if (copy_node3)
					printf(" -> NOT COPIED: %s\n", n->name);
			}
		}
	}
}

void process_Building_ReliefFeature_textures(xmlNodePtr noeud, std::set<std::string> *UUID_s, std::string folderIN, std::string folderOUT)
{
	if (noeud->type == XML_ELEMENT_NODE)
	{
        xmlChar *chemin = xmlGetNodePath(noeud);
        if (noeud->children != NULL)// && noeud->children->type == XML_TEXT_NODE) // MT
		{
            xmlChar *contenu = xmlNodeGetContent(noeud);
			if (xmlStrEqual(noeud->name, BAD_CAST "appearance"))
			{
				process_textures(noeud, NULL, UUID_s, folderIN, folderOUT);
			}
            xmlFree(contenu);
        }
		else
		{
			//printf("%s\n", chemin);
        }
        xmlFree(chemin);
    }
}

typedef void (*fct_process_Building_ReliefFeature_textures)(xmlNodePtr, std::set<std::string> *, std::string, std::string);
void parcours_prefixe_Building_ReliefFeature_textures(xmlNodePtr noeud, fct_process_Building_ReliefFeature_textures f, std::set<std::string> *UUID_s, std::string folderIN, std::string folderOUT)
{
	xmlNodePtr n;
    
    for (n = noeud; n != NULL; n = n->next)
	{
        f(n, UUID_s, folderIN, folderOUT);

        if ((n->type == XML_ELEMENT_NODE) && (n->children != NULL))
		{
            parcours_prefixe_Building_ReliefFeature_textures(n->children, f, UUID_s, folderIN, folderOUT);
        }
    }
}

int main(int argc, char** argv)
{
	if (argc != 7)
	{
		puts("");
        puts("ParseCityGML 1.0.9 - June 25, 2014 - Martial TOLA");
		puts("-> this tool parses a CityGML file according to a 2d bounding box and extracts Buildings, ReliefFeatures and corresponding surfaceDataMembers.");
		puts("Usage:");
		puts("");
		puts("ParseCityGML <file-to-parse> <output-file> <xmin> <ymin> <xmax> <ymax>");
		puts("");
		puts("Example:");
		puts("./parseCityGML ZoneAExporter.gml outP.gml 643200 6861700 643300 6861800");
		puts("./parseCityGML LYON_3.gml outL.gml 1843000 5174000 1844000 5175000");
		puts("");
    
		return(EXIT_FAILURE);
	}

	std::string folderIN, folderOUT;
	int nbCopied = 0;
	bool POST_PROCESS_TEXTURES = false;
	xmlNodePtr appearanceMember_node = NULL;
	std::set<std::string> UUID_full_set;

	G_xmin = atof(argv[3]);
	G_ymin = atof(argv[4]);
	G_xmax = atof(argv[5]);
	G_ymax = atof(argv[6]);

	if (! ((G_xmin < G_xmax) && (G_ymin < G_ymax)) )
		return(EXIT_FAILURE);

	// ---
	// 4 planes (normal and point)
	G_my4planes.n[0].x=0;					G_my4planes.n[0].y=-1;					G_my4planes.n[0].z=0; // bottom plane
	G_my4planes.p0[0].x=(G_xmin+G_xmax)/2.;	G_my4planes.p0[0].y=G_ymin;				G_my4planes.p0[0].z=0;

	G_my4planes.n[1].x=-1;					G_my4planes.n[1].y=0;					G_my4planes.n[1].z=0; // left plane
	G_my4planes.p0[1].x=G_xmin;				G_my4planes.p0[1].y=(G_ymin+G_ymax)/2.;	G_my4planes.p0[1].z=0;

	G_my4planes.n[2].x=0;					G_my4planes.n[2].y=1;					G_my4planes.n[2].z=0; // up plane
	G_my4planes.p0[2].x=(G_xmin+G_xmax)/2.;	G_my4planes.p0[2].y=G_ymax;				G_my4planes.p0[2].z=0;

	G_my4planes.n[3].x=1;					G_my4planes.n[3].y=0;					G_my4planes.n[3].z=0; // right plane
	G_my4planes.p0[3].x=G_xmax;				G_my4planes.p0[3].y=(G_ymin+G_ymax)/2.;	G_my4planes.p0[3].z=0;

	for (int p=0; p<4; p++)
		G_my4planes.n[p]=G_my4planes.n[p].normal(); // normalizing
	// ---

    xmlDocPtr out_doc = NULL;			// output document pointer
    xmlNodePtr out_root_node = NULL;	// output root node pointer

	// creates a new document
    out_doc = xmlNewDoc(BAD_CAST "1.0");

    xmlDocPtr doc;
    xmlNodePtr racine;
 
    // opens document
    xmlKeepBlanksDefault(0); // ignore les noeuds texte composant la mise en forme
    doc = xmlParseFile(argv[1]);
    if (doc == NULL)
	{
        fprintf(stderr, "Invalid XML file\n");
        return EXIT_FAILURE;
    }

	QFileInfo fiIN(argv[1]);
	folderIN = fiIN.absolutePath().toStdString();
	std::cout << " -> folderIN: " << folderIN << std::endl;

	QFileInfo fiOUT(argv[2]);
	folderOUT = fiOUT.absolutePath().toStdString();
	std::cout << " -> folderOUT: " << folderOUT << std::endl;
	QDir dir(fiOUT.absolutePath());
	if (!dir.exists())
		dir.mkpath(".");

    // get root
    racine = xmlDocGetRootElement(doc);
    if (racine == NULL)
	{
        fprintf(stderr, "Empty XML file\n");
        xmlFreeDoc(doc);
        return EXIT_FAILURE;
    }

    // parcours
	xmlNodePtr n = racine;
	if (xmlStrEqual(n->name, BAD_CAST "CityModel"))
	{
		printf("%s\n", n->name);
		xmlNodePtr copy_node1 = xmlCopyNode(n, 2);
		xmlDocSetRootElement(out_doc, copy_node1);
		out_root_node = xmlDocGetRootElement(out_doc);

		if ((n->type == XML_ELEMENT_NODE) && (n->children != NULL))
		{
			for (n = n->children; n != NULL; n = n->next)
			{
				if (xmlStrEqual(n->name, BAD_CAST "cityObjectMember"))
				{
					/*if (xmlStrEqual(n->children->name, BAD_CAST "Building"))
					{
						if (xmlStrEqual(n->children->children->name, BAD_CAST "stringAttribute"))
							if (xmlStrEqual(xmlGetProp(n->children->children, BAD_CAST "name"), BAD_CAST "centre"))
								if (xmlStrEqual(n->children->children->children->name, BAD_CAST "value"))
								{
									double x, y;
									char *endptr;
									xmlChar *contenu = xmlNodeGetContent(n->children->children->children);
										x = strtod((char *) contenu, &endptr);
										y = strtod(endptr, NULL);
									xmlFree(contenu);

									if ( (x >= xmin) && (x <= xmax) )
										if ( (y >= ymin) && (y <= ymax) )
										{
											printf("%s: %s - %s (centre: %lf %lf)\n", n->name, n->children->name, xmlGetProp(n->children, BAD_CAST "id"), x, y);

											//xmlNs ns = { 0 }; // initialisation à zéro de tous les membres
											xmlNodePtr copy_node2 = xmlCopyNode(n, 1);
											//xmlSetNs(copy_node2, &ns); //xmlSetNs(copy_node2, NULL);

											xmlAddChild(out_root_node, copy_node2);
										}
								}
					}*/
					if ( (xmlStrEqual(n->children->name, BAD_CAST "Building")) || (xmlStrEqual(n->children->name, BAD_CAST "ReliefFeature")) ) // ReliefFeature same principle as Building
					//if (xmlStrEqual(n->children->name, BAD_CAST "Building")) // only Building
					//if (xmlStrEqual(n->children->name, BAD_CAST "ReliefFeature")) // only ReliefFeature
					{
						double xmin_Building, ymin_Building, zmin_Building;
						double xmax_Building, ymax_Building, zmax_Building;
						xmin_Building = ymin_Building = zmin_Building = xmax_Building = ymax_Building = zmax_Building = 0.;
						std::set<std::string> UUID_set;

						bool first=true;
						parcours_prefixe_Building_ReliefFeature_boundingbox(n->children, process_Building_ReliefFeature_boundingbox, &first, &xmin_Building, &ymin_Building, &zmin_Building, &xmax_Building, &ymax_Building, &zmax_Building, &UUID_set);
						//printf("\nMIN_Building: (%lf %lf %lf)\n", xmin_Building, ymin_Building, zmin_Building);
						//printf("MAX_Building: (%lf %lf %lf)\n", xmax_Building, ymax_Building, zmax_Building);

						//exit(-1);
						/*if ( (xmin_Building >= G_xmin) && (xmax_Building <= G_xmax) )
							if ( (ymin_Building >= G_ymin) && (ymax_Building <= G_ymax) )*/
                        if ( !(xmax_Building < G_xmin) && !(ymax_Building < G_ymin) && !(xmin_Building > G_xmax) && !(ymin_Building > G_ymax) )
							{
								printf("%s: %s - %s (min: %lf %lf) (max: %lf %lf)\n", n->name, n->children->name, xmlGetProp(n->children, BAD_CAST "id"), xmin_Building, ymin_Building, xmax_Building, ymax_Building);

								//xmlNs ns = { 0 }; // initialisation à zéro de tous les membres
								xmlNodePtr copy_node2 = xmlCopyNode(n, 1);
								//xmlSetNs(copy_node2, &ns); //xmlSetNs(copy_node2, NULL);

								xmlAddChild(out_root_node, copy_node2);
								nbCopied++;

								if (TEXTURE_PROCESS)
								{
									for (std::set<std::string>::iterator it=UUID_set.begin(); it!=UUID_set.end(); ++it)
										if ( UUID_full_set.find(*it) == UUID_full_set.end() )
											UUID_full_set.insert(*it);
										/*else
											printf("FOUND in UUID_full_set\n");*/

									//printf("parcours_prefixe_Building_ReliefFeature_textures: %s - %s\n", n->children->name, xmlGetProp(n->children, BAD_CAST "id"));
									parcours_prefixe_Building_ReliefFeature_textures(n->children, process_Building_ReliefFeature_textures, &UUID_set, folderIN, folderOUT);
								}
							}
					}					
					else
						printf(" -> NOT COPIED: %s: %s\n", n->name, n->children->name);
				}
				else if ( (xmlStrEqual(n->name, BAD_CAST "appearanceMember")) && (TEXTURE_PROCESS==true) )
				{
					POST_PROCESS_TEXTURES = true;
					appearanceMember_node = n; // CAUTION : FOR NOW, WE SUPPOSE ONLY ONE appearanceMember
					printf(" -> POST PROCESS TEXTURES AFTER ALL PARSING: %s\n", n->name);
				}
				else
					printf(" -> NOT COPIED: %s\n", n->name);
			}
		}

		if (POST_PROCESS_TEXTURES)
		{
			printf("\nPOST PROCESS TEXTURES\n");

			n = appearanceMember_node; // CAUTION : FOR NOW, WE SUPPOSE ONLY ONE appearanceMember
			xmlNodePtr copy_node3 = xmlCopyNode(n, 2);
			xmlAddChild(out_root_node, copy_node3);

			process_textures(n, copy_node3, &UUID_full_set, folderIN, folderOUT);
		}
	}

	printf(" -> NB COPIED: %d\n", nbCopied);

    // dumping document to file
    xmlSaveFormatFileEnc(argv[2], out_doc, "ISO-8859-1", 1);

    // free the documents
    xmlFreeDoc(out_doc);
    xmlFreeDoc(doc);

    return EXIT_SUCCESS;
}
