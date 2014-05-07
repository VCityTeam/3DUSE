#include <stdio.h>
#include <stdlib.h>
#include <libxml/tree.h>
#include <libxml/parser.h>

int main(int argc, char** argv)
{
	if (argc != 7)
	{
		puts("");
		puts("ParseCityGML 1.0.0 - May 7, 2014 - Martial TOLA");
		puts("-> this tiny tool parses a CityGML file according to a 2d bounding box and extracts buildings.");
		puts("Usage:");
		puts("");
		puts("ParseCityGML <file-to-parse> <output-file> <xmin> <ymin> <xmax> <ymax>");
		puts("");
		puts("Example: ./parseCityGML ZoneAExporter.gml out.gml 643200 6861700 643300 6861800");
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
					{
						if (xmlStrEqual(n->children->children->name, BAD_CAST "stringAttribute"))
							if (xmlStrEqual(xmlGetProp(n->children->children, BAD_CAST "name"), BAD_CAST "centre"))
								if (xmlStrEqual(n->children->children->children->name, BAD_CAST "value"))
								{
									double x, y;
									char *endptr;
									xmlChar *contenu = xmlNodeGetContent(n->children->children->children);
										x = strtod((char *) contenu, &endptr);
										y = strtod((char *) endptr, NULL);
									xmlFree(contenu);

									if ( (x >= xmin) && (x <= xmax) )
										if ( (y >= ymin) && (y <= ymax) )
										{
											printf("%s: %s - %s (%lf %lf)\n", n->name, n->children->name, xmlGetProp(n->children, BAD_CAST "id"), x, y);

											//xmlNs ns = { 0 }; // initialisation à zéro de tous les membres
											xmlNodePtr copy_node2 = xmlCopyNode(n, 1);
											//xmlSetNs(copy_node2, &ns); //xmlSetNs(copy_node2, NULL);

											xmlAddChild(out_root_node, copy_node2);
										}
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