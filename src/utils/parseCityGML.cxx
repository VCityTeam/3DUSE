#include <stdio.h>
#include <stdlib.h>
#include <libxml/tree.h>
#include <libxml/parser.h>

void afficher_noeud(xmlNodePtr noeud, bool *first_posList, double *xmin, double *ymin, double *zmin, double *xmax, double *ymax, double *zmax)
{
	if (noeud->type == XML_ELEMENT_NODE)
	{
        xmlChar *chemin = xmlGetNodePath(noeud);
        if (noeud->children != NULL && noeud->children->type == XML_TEXT_NODE)
		{
            xmlChar *contenu = xmlNodeGetContent(noeud);
			if (xmlStrEqual(noeud->name, BAD_CAST "posList"))
			{
				//printf("%s -> %s\n", chemin, contenu);
				//printf("posList: %s\n", contenu);

				double x, y, z;
				char *endptr = NULL;

				bool first = TRUE;
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
					first = FALSE;
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
				*first_posList = FALSE;
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

typedef void (*fct_parcours_t)(xmlNodePtr, bool *, double *, double *, double *, double *, double *, double *);

void parcours_prefixe(xmlNodePtr noeud, fct_parcours_t f, bool *first, double *xmin, double *ymin, double *zmin, double *xmax, double *ymax, double *zmax)
{
    xmlNodePtr n;
    
    for (n = noeud; n != NULL; n = n->next)
	{
        f(n, first, xmin, ymin, zmin, xmax, ymax, zmax);

        if ((n->type == XML_ELEMENT_NODE) && (n->children != NULL))
		{
            parcours_prefixe(n->children, f, first, xmin, ymin, zmin, xmax, ymax, zmax);
        }
    }
}

int main(int argc, char** argv)
{
	if (argc != 7)
	{
		puts("");
		puts("ParseCityGML 1.0.1 - May 21, 2014 - Martial TOLA");
		puts("-> this tiny tool parses a CityGML file according to a 2d bounding box and extracts buildings.");
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

	double xmin = atof(argv[3]);
	double ymin = atof(argv[4]);
	double xmax = atof(argv[5]);
	double ymax = atof(argv[6]);

	if (! ((xmin < xmax) && (ymin < ymax)) )
		return(EXIT_FAILURE);

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
					if (xmlStrEqual(n->children->name, BAD_CAST "Building"))
					/*{
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
					{
						double xmin_Building, ymin_Building, zmin_Building;
						double xmax_Building, ymax_Building, zmax_Building;
						xmin_Building = ymin_Building = zmin_Building = xmax_Building = ymax_Building = zmax_Building = 0.;

						bool first=TRUE;
						parcours_prefixe(n->children, afficher_noeud, &first, &xmin_Building, &ymin_Building, &zmin_Building, &xmax_Building, &ymax_Building, &zmax_Building);
						//printf("\nMIN_Building: (%lf %lf %lf)\n", xmin_Building, ymin_Building, zmin_Building);
						//printf("MAX_Building: (%lf %lf %lf)\n", xmax_Building, ymax_Building, zmax_Building);

						//exit(-1);
						if ( (xmin_Building >= xmin) && (xmax_Building <= xmax) )
							if ( (ymin_Building >= ymin) && (ymax_Building <= ymax) )
							{
								printf("%s: %s - %s (min: %lf %lf) (max: %lf %lf)\n", n->name, n->children->name, xmlGetProp(n->children, BAD_CAST "id"), xmin_Building, ymin_Building, xmax_Building, ymax_Building);

								//xmlNs ns = { 0 }; // initialisation à zéro de tous les membres
								xmlNodePtr copy_node2 = xmlCopyNode(n, 1);
								//xmlSetNs(copy_node2, &ns); //xmlSetNs(copy_node2, NULL);

								xmlAddChild(out_root_node, copy_node2);
							}
					}
					else
						printf(" -> NOT COPIED: %s: %s\n", n->name, n->children->name);
				}
				else
					printf(" -> NOT COPIED: %s\n", n->name);
			}
		}
	}

    // dumping document to file
    xmlSaveFormatFileEnc(argv[2], out_doc, "ISO-8859-1", 1);

    // free the documents
    xmlFreeDoc(out_doc);
    xmlFreeDoc(doc);

    return EXIT_SUCCESS;
}