// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "exportCityGML.hpp"
#include <libxml/parser.h>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
	////////////////////////////////////////////////////////////////////////////////
	ExporterCityGML::ExporterCityGML(const std::string& filename)
		: m_fileName(filename), m_doc(nullptr), m_root_node(nullptr), m_currentAppearence(nullptr)
	{
	}
	////////////////////////////////////////////////////////////////////////////////
	ExporterCityGML::~ExporterCityGML()
	{
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::initExport(bool createCityModelRootNode)
	{
		m_doc = nullptr;
		m_root_node = nullptr;

		LIBXML_TEST_VERSION;

		/*
		* Creates a new document, a node and set it as a root node
		*/
		m_doc = xmlNewDoc(BAD_CAST "1.0");
		if(createCityModelRootNode)
		{
			//m_root_node = xmlNewNode(NULL, BAD_CAST "core:CityModel");
			m_root_node = xmlNewNode(NULL, BAD_CAST "CityModel");
			xmlNewProp(m_root_node, BAD_CAST "xmlns", BAD_CAST "http://www.opengis.net/citygml/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:app", BAD_CAST "http://www.opengis.net/citygml/appearance/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:bldg", BAD_CAST "http://www.opengis.net/citygml/building/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:core", BAD_CAST "http://www.opengis.net/citygml/base/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:dem", BAD_CAST "http://www.opengis.net/citygml/relief/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:tran", BAD_CAST "http://schemas.opengis.net/citygml/transportation/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:wtr", BAD_CAST "http://www.opengis.net/citygml/waterbody/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:gen", BAD_CAST "http://www.opengis.net/citygml/generics/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:gml", BAD_CAST "http://www.opengis.net/gml");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:tex", BAD_CAST "http://www.opengis.net/citygml/textures/1.0");
			xmlNewProp(m_root_node, BAD_CAST "xmlns:xsi", BAD_CAST "http://www.w3.org/2001/XMLSchema-instance");
			xmlNewProp(m_root_node, BAD_CAST "xsi:schemaLocation", BAD_CAST "http://www.opengis.net/citygml/building/1.0 http://schemas.opengis.net/citygml/building/1.0/building.xsd http://www.opengis.net/citygml/appearance/1.0 http://schemas.opengis.net/citygml/appearance/1.0/appearance.xsd http://www.opengis.net/citygml/relief/1.0 http://schemas.opengis.net/citygml/relief/1.0/relief.xsd");
		}
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::endExport()
	{
		xmlDocSetRootElement(m_doc, m_root_node);

		/*
		* Dumping document to stdio or file
		*/
		xmlSaveFormatFileEnc(m_fileName.c_str(), m_doc, "UTF-8", 1);

		/*free the document */
		xmlFreeDoc(m_doc);

		/*
		*Free the global variables that may
		*have been allocated by the parser.
		*/
		xmlCleanupParser();

		/*
		* this is to debug memory for regression tests
		*/
		xmlMemoryDump();
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::addEnvelope(const citygml::Envelope& env)
	{
		xmlNodePtr nodeEnv = xmlNewChild(m_root_node, NULL, BAD_CAST "gml:boundedBy", NULL);
		exportEnvelopeXml(env, nodeEnv);
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::exportCityModel(const CityModel& model)
	{
		initExport(false);

		m_root_node = exportCityModelXml(model);

		endExport();
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::exportCityModelWithListTextures(const CityModel& model, std::vector<TextureCityGML*>* ListTextures)
	{
		initExport(false);

		m_root_node = exportCityModelXml(model);

		if(ListTextures->size() > 0)
			exportListTextures(m_root_node, ListTextures);

		endExport();
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::exportCityObject(const std::vector<const CityObject*>& models)
	{
		initExport(false);

		m_root_node = exportCityObjectModelXml(models);

		endExport();
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::exportCityObjectWithListTextures(const std::vector<const CityObject*>& models, std::vector<TextureCityGML*>* ListTextures)
	{
		initExport(false);

		m_root_node = exportCityObjectModelXml(models);

		if(ListTextures->size() > 0)
			exportListTextures(m_root_node, ListTextures);

		endExport();
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::appendCityObject(const std::vector<const CityObject*>& objs)
	{
		for(const CityObject* obj : objs)
		{
			appendCityObject(*obj);
		}
	}
	////////////////////////////////////////////////////////////////////////////////
	void ExporterCityGML::appendCityObject(const CityObject& obj)
	{
		//xmlNodePtr node = xmlNewChild(m_root_node, NULL, BAD_CAST "core:cityObjectMember", NULL);
		xmlNodePtr node = xmlNewChild(m_root_node, NULL, BAD_CAST "cityObjectMember", NULL);
		exportCityObjetXml(obj, node);
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportEnvelopeXml(const citygml::Envelope& env, xmlNodePtr parent)
	{
		xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST "gml:Envelope", NULL);

        xmlNewProp(res, BAD_CAST "srsDimension", BAD_CAST "3");
        xmlNewProp(res, BAD_CAST "srsName", BAD_CAST "EPSG:3946"); /// VALABLE QUE SUR LYON !!!!!!!!!

		std::stringstream ss;
		TVec3d vec = env.getLowerBound();
		ss << std::fixed << vec.x << " " << vec.y << " " << vec.z;
		xmlNewChild(res, NULL, BAD_CAST "gml:lowerCorner", BAD_CAST ss.str().c_str());
		//ss.clear();
		ss.str(std::string());
		vec = env.getUpperBound();
		ss << std::fixed << vec.x << " " << vec.y << " " << vec.z;
		xmlNewChild(res, NULL, BAD_CAST "gml:upperCorner", BAD_CAST ss.str().c_str());

		return res;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportLinearRingXml(const citygml::LinearRing& ring, xmlNodePtr parent)
	{
		xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST "gml:LinearRing", NULL);
		xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST ring.getId().c_str());

		std::stringstream ss;
		for(const TVec3d& v : ring.getVertices())
		{
			ss << std::fixed << v.x << " " << v.y << " " << v.z << " ";
		}
		const TVec3d& v = ring.getVertices().front();
		ss << std::fixed << v.x << " " << v.y << " " << v.z;
		xmlNodePtr pos = xmlNewChild(res, NULL, BAD_CAST "gml:posList", BAD_CAST ss.str().c_str());
		
		xmlNewProp(pos, BAD_CAST "srsDimension", BAD_CAST "3");

		return res;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportPolygonXml(const citygml::Polygon& poly, xmlNodePtr parent)
	{
		xmlNodePtr res;
		xmlNodePtr node = xmlNewChild(parent, NULL, BAD_CAST "gml:surfaceMember", NULL);
		res = xmlNewChild(node, NULL, BAD_CAST ("gml:Polygon"), NULL);
		xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST poly.getId().c_str());

		const std::vector<citygml::LinearRing*>& lrings = poly.getInteriorRings();
		std::vector<citygml::LinearRing*>::const_iterator it = lrings.begin();
		for(; it != lrings.end(); ++it)
		{
			xmlNodePtr node = xmlNewChild(res, NULL, BAD_CAST "gml:interior", NULL);
			exportLinearRingXml(**it, node);
		}

		if(poly.getExteriorRing())
		{
			xmlNodePtr node = xmlNewChild(res, NULL, BAD_CAST "gml:exterior", NULL);
			exportLinearRingXml(*poly.getExteriorRing(), node);
		}

		return res;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportGeometryGenericXml(const citygml::Geometry& geom, const std::string& nodeType, xmlNodePtr parent)
	{
        //xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST nodeType.c_str(), NULL); //Maxime a ajouté un Wall (Roof) après le WallSurface (RoofSurface) alors qu'il n'y est pas dans les CityGML fournis
        //xmlNewChild(res, NULL, BAD_CAST "gml:name", BAD_CAST geom.getId().c_str());

       // xmlNodePtr node1 = xmlNewChild(parent, NULL, BAD_CAST (std::string("bldg:lod")+std::to_string(geom.getLOD())+"MultiSurface").c_str(), NULL);
       // xmlNodePtr node2 = xmlNewChild(node1, NULL, BAD_CAST "gml:MultiSurface", NULL);
       // xmlNewProp(node2, BAD_CAST "srsDimension", BAD_CAST "3");
       // xmlNodePtr node3 = xmlNewChild(parent, NULL, BAD_CAST "gml:surfaceMember", NULL);

		for(const citygml::Polygon* poly : geom.getPolygons())
		{
			//exportPolygonAppearanceXml(*poly, m_currentAppearence); ///////// EXPORT TEXTURE VERSION MAXIME -> Un appel du fichier image par Polygon. Commenté car texture gérée par exportCityModelWithListTextures.
			exportPolygonXml(*poly, parent);//node3
		}

        //return res;
        return parent;//node1;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportGeometryXml(const citygml::Geometry& geom, xmlNodePtr parent)
	{
		switch(geom.getType())
		{
		case citygml::GT_Unknown:
			return exportGeometryGenericXml(geom, "bldg:Unknown", parent);
			break;
		case citygml::GT_Roof:
			return exportGeometryGenericXml(geom, "bldg:Roof", parent);
			break;
		case citygml::GT_Wall:
			return exportGeometryGenericXml(geom, "bldg:Wall", parent);
			break;
		case citygml::GT_Ground:
			return exportGeometryGenericXml(geom, "bldg:Ground", parent);
			break;
		case citygml::GT_Closure:
			return exportGeometryGenericXml(geom, "bldg:Closure", parent);
			break;
		case citygml::GT_Floor:
			return exportGeometryGenericXml(geom, "bldg:Floor", parent);
			break;
		case citygml::GT_InteriorWall:
			return exportGeometryGenericXml(geom, "bldg:InteriorWall", parent);
			break;
		case citygml::GT_Ceiling:
			return exportGeometryGenericXml(geom, "bldg:Ceiling", parent);
			break;
		default:
			break;
		}

		return NULL;
	}
	////////////////////////////////////////////////////////////////////////////////
    xmlNodePtr ExporterCityGML::exportCityObjetGenericXml(const citygml::CityObject& obj, const std::string& nodeType, xmlNodePtr parent, bool isSurface)
	{
		if (obj._isXlink==xLinkState::LINKED)
		{
			xmlNodePtr res;
			if (isSurface) 
			{
				std::string ns = nodeType.substr(0,nodeType.find_first_of(":"));
				xmlNodePtr nodebb = xmlNewChild(parent, NULL, BAD_CAST (ns+":boundedBy").c_str(), NULL);
				res = xmlNewChild(nodebb, NULL, BAD_CAST nodeType.c_str(), NULL);
			} 
			else res = xmlNewChild(parent, NULL, BAD_CAST nodeType.c_str(), NULL);
			std::string query = obj.getAttribute("xlink");
			xmlNewProp(res, BAD_CAST "xlink:href", BAD_CAST query.c_str());
			return res;
		}
        if(isSurface)
        {
			std::string ns = nodeType.substr(0,nodeType.find_first_of(":"));
			xmlNodePtr nodebb = xmlNewChild(parent, NULL, BAD_CAST (ns+":boundedBy").c_str(), NULL);// Ajouté car présent dans les CityGML de Lyon et Paris

            xmlNodePtr res = xmlNewChild(nodebb, NULL, BAD_CAST nodeType.c_str(), NULL);
            xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST obj.getId().c_str());

            // children are not handled here, but in the upper call level (exportCityObjetXml)
            /*for(const auto& child : obj.getChildren())
            {
            exportCityObjetXml(child, res);
            }*/

            // attributes
            for(const auto& attr : obj.getAttributes())
            {
				std::string attrName = attr.first; 
				if(attrName=="yearOfConstruction"||attrName=="yearOfDemolition")
				{
					std::string cName = "bldg:"+attrName;
					xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST cName.c_str(), BAD_CAST attr.second.c_str());
				}
				else if (attrName=="creationDate"||attrName=="terminationDate")
				{					
					std::string cName = "core:"+attrName;
					xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST cName.c_str(), BAD_CAST attr.second.c_str());
				}
				else if (attrName=="identifier") xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST ("gml:"+attrName).c_str(), BAD_CAST attr.second.c_str());
				else
				{
					xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST "gen:stringAttribute", NULL);
					xmlNewProp(attrNode, BAD_CAST "name", BAD_CAST attr.first.c_str());
					xmlNewChild(attrNode, NULL, BAD_CAST "gen:value", BAD_CAST attr.second.c_str());
				}
            }

            return res;
        }
        else
        {
            xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST nodeType.c_str(), NULL);
            xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST obj.getId().c_str());

            // children are not handled here, but in the upper call level (exportCityObjetXml)
            /*for(const auto& child : obj.getChildren())
            {
            exportCityObjetXml(child, res);
            }*/

            // attributes
            for(const auto& attr : obj.getAttributes())
            {
				std::string attrName = attr.first; 
				if(attrName=="yearOfConstruction"||attrName=="yearOfDemolition")
				{
					std::string cName = "bldg:"+attrName;
					xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST cName.c_str(), BAD_CAST attr.second.c_str());
				}
				else if (attrName=="creationDate"||attrName=="terminationDate")
				{					
					std::string cName = "core:"+attrName;
					xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST cName.c_str(), BAD_CAST attr.second.c_str());
				}
				else if (attrName=="identifier") xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST ("gml:"+attrName).c_str(), BAD_CAST attr.second.c_str());
				else
				{
                xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST "gen:stringAttribute", NULL);
                xmlNewProp(attrNode, BAD_CAST "name", BAD_CAST attr.first.c_str());
                xmlNewChild(attrNode, NULL, BAD_CAST "gen:value", BAD_CAST attr.second.c_str());
				}
            }

            return res;
        }

	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportCityObjetStateXml(const citygml::CityObjectState& state, const std::string &nodeType, xmlNodePtr parent)
	{
		xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST nodeType.c_str(), NULL);
		xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST (state.getParent()->getId()+'_'+state.getStringId()).c_str());

		if(state.getGeom())
		{
			// attributes
			for(const auto& attr : state.getGeom()->getAttributes())
			{
				xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST "gen:stringAttribute", NULL);
				xmlNewProp(attrNode, BAD_CAST "name", BAD_CAST attr.first.c_str());
				xmlNewChild(attrNode, NULL, BAD_CAST "gen:value", BAD_CAST attr.second.c_str());
			}
		}

		return res;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportCityObjetTagXml(const citygml::CityObjectTag& tag, const std::string &nodeType, xmlNodePtr parent)
	{
		xmlNodePtr res = xmlNewChild(parent, NULL, BAD_CAST nodeType.c_str(), NULL);
		xmlNewProp(res, BAD_CAST "gml:id", BAD_CAST (tag.getParent()->getId()+'_'+tag.getStringId()).c_str());

		if(tag.getGeom())
		{
			// attributes
			for(const auto& attr : tag.getGeom()->getAttributes())
			{
				xmlNodePtr attrNode = xmlNewChild(res, NULL, BAD_CAST "gen:stringAttribute", NULL);
				xmlNewProp(attrNode, BAD_CAST "name", BAD_CAST attr.first.c_str());
				xmlNewChild(attrNode, NULL, BAD_CAST "gen:value", BAD_CAST attr.second.c_str());
			}
		}

		// date attrib
		xmlNodePtr dateNode = xmlNewChild(res, NULL, BAD_CAST "gen:dateAttribute", NULL);
		xmlNewProp(dateNode, BAD_CAST "name", BAD_CAST "date");
		xmlNewChild(dateNode, NULL, BAD_CAST "gen:value", BAD_CAST tag.m_date.toString(Qt::ISODate).toStdString().c_str());

		return res;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportCityObjetXml(const citygml::CityObject& obj, xmlNodePtr parent, bool rootLevel)
	{
		xmlNodePtr res = NULL;

		//std::cout << "type : " << obj.getTypeAsString() << std::endl;
		
		// temporal test
		if(m_temporalExport)
		{
			std::string strAttr = obj.getAttribute("yearOfConstruction");
			int yearOfConstruction = (strAttr.empty()?-1:std::stoi(strAttr));
			strAttr = obj.getAttribute("yearOfDemolition");
			int yearOfDemolition = (strAttr.empty()?-1:std::stoi(strAttr));

			int year = m_date.date().year();

			if(((yearOfConstruction == -1 || yearOfDemolition == -1) || (yearOfConstruction < year && year < yearOfDemolition)))
			{
				// keep node
			}
			else
			{
				return res;
			}

			// advanced temporal test

		}

		std::string type = "bldg:";

		switch(obj.getType())
		{
		case citygml::COT_GenericCityObject:
			break;
		case citygml::COT_Building:
			res = exportCityObjetGenericXml(obj, "bldg:Building", parent);
			break;
		case citygml::COT_Room:
			res = exportCityObjetGenericXml(obj, "bldg:Room", parent);
			break;
		case citygml::COT_BuildingInstallation:
			res = exportCityObjetGenericXml(obj, "bldg:BuildingInstallation", parent);
			break;
		case citygml::COT_BuildingFurniture:
			res = exportCityObjetGenericXml(obj, "bldg:BuildingFurniture", parent);
			break;
		case citygml::COT_Door:
			res = exportCityObjetGenericXml(obj, "bldg:Door", parent);
			break;
		case citygml::COT_Window:
			res = exportCityObjetGenericXml(obj, "bldg:Window", parent);
			break;
		case citygml::COT_CityFurniture:
			res = exportCityObjetGenericXml(obj, "bldg:CityFurniture", parent);
			break;
		case citygml::COT_Track:
			res = exportCityObjetGenericXml(obj, "tran:Track", parent);
			type = "tran:";
			break;
		case citygml::COT_Road:
			res = exportCityObjetGenericXml(obj, "tran:Road", parent);
			type = "tran:";
			break;
		case citygml::COT_Railway:
			res = exportCityObjetGenericXml(obj, "tran:Railway", parent);
			type = "tran:";
			break;
		case citygml::COT_Square:
			res = exportCityObjetGenericXml(obj, "bldg:Square", parent);
			break;
		case citygml::COT_PlantCover:
			res = exportCityObjetGenericXml(obj, "bldg:PlantCover", parent);
			break;
		case citygml::COT_SolitaryVegetationObject:
			res = exportCityObjetGenericXml(obj, "bldg:SolitaryVegetationObject", parent);
			break;
		case citygml::COT_WaterBody:
			res = exportCityObjetGenericXml(obj, "wtr:WaterBody", parent);
			type = "wtr:";
			break;
		case citygml::COT_TINRelief:
			{
				xmlNodePtr node1 = xmlNewChild(parent, NULL, BAD_CAST "dem:ReliefFeature", NULL);
				std::string id;
				std::stringstream ss; ss << "PtrId_" << node1; id = ss.str();
				xmlNewProp(node1, BAD_CAST "gml:id", BAD_CAST id.c_str());
				xmlNodePtr node2 = xmlNewChild(node1, NULL, BAD_CAST "dem:reliefComponent", NULL);
				res = exportCityObjetGenericXml(obj, "dem:TINRelief", node2);
			}
			type = "dem:";
			break;
		case citygml::COT_LandUse:
			res = exportCityObjetGenericXml(obj, "bldg:LandUse", parent);
			break;
		case citygml::COT_Tunnel:
			res = exportCityObjetGenericXml(obj, "tran:Tunnel", parent);
			type = "tran:";
			break;
		case citygml::COT_Bridge:
			res = exportCityObjetGenericXml(obj, "tran:Bridge", parent);
			type = "tran:";
			break;
		case citygml::COT_BridgeConstructionElement:
			res = exportCityObjetGenericXml(obj, "tran:BridgeConstructionElement", parent);
			type = "tran:";
			break;
		case citygml::COT_BridgeInstallation:
			res = exportCityObjetGenericXml(obj, "tran:BridgeInstallation", parent);
			type = "tran:";
			break;
		case citygml::COT_BridgePart:
			res = exportCityObjetGenericXml(obj, "tran:BridgePart", parent);
			type = "tran:";
			break;
		case citygml::COT_BuildingPart:
			res = exportCityObjetGenericXml(obj, "bldg:BuildingPart", parent);
			break;
		case citygml::COT_WallSurface:
            res = exportCityObjetGenericXml(obj, "bldg:WallSurface", parent, true);
			break;
		case citygml::COT_RoofSurface:
            res = exportCityObjetGenericXml(obj, "bldg:RoofSurface", parent, true);
			break;
		case citygml::COT_GroundSurface:
            res = exportCityObjetGenericXml(obj, "bldg:GroundSurface", parent, true);
			break;
		case citygml::COT_ClosureSurface:
            res = exportCityObjetGenericXml(obj, "bldg:ClosureSurface", parent, true);
			break;
		case citygml::COT_FloorSurface:
            res = exportCityObjetGenericXml(obj, "bldg:FloorSurface", parent, true);
			break;
		case citygml::COT_InteriorWallSurface:
            res = exportCityObjetGenericXml(obj, "bldg:InteriorWallSurface", parent, true);
			break;
		case citygml::COT_CeilingSurface:
            res = exportCityObjetGenericXml(obj, "bldg:CeilingSurface", parent, true);
			break;
		default:
			break;
		}

		if(!m_temporalExport) // export TAGs and STATEs
		{   
			for(CityObjectState* state : obj.getStates())
			{
				xmlNodePtr r = exportCityObjetStateXml(*state, type + state->getParent()->getTypeAsString(), parent);

				// build apperance node for current node
				/*if(rootLevel)
				{
					m_currentAppearence = xmlNewChild(r, NULL, BAD_CAST "app:appearance", NULL);
				}*/

				if(state->getGeom())
				{
					for(const auto& geom : state->getGeom()->getGeometries())
					{
						exportGeometryXml(*geom, r);
					}

					for(const auto& child : state->getGeom()->getChildren())
					{
						exportCityObjetXml(*child, r);
					}
				}
			}
			for(CityObjectTag* tag : obj.getTags())
			{
				xmlNodePtr r = exportCityObjetTagXml(*tag, type + tag->getParent()->getTypeAsString(), parent);

				// build apperance node for current node
				/*if(rootLevel)
				{
					m_currentAppearence = xmlNewChild(r, NULL, BAD_CAST "app:appearance", NULL);
				}*/

				if(tag->getGeom())
				{
					for(const auto& geom : tag->getGeom()->getGeometries())
					{
						exportGeometryXml(*geom, r);
					}

					for(const auto& child : tag->getGeom()->getChildren())
					{
						exportCityObjetXml(*child, r);
					}
				}
			}
		}

		// build apperance node for current node //// F.pedrinis 10/03/16 : Utilité d'un <app:appearance/> vide à chaque city object ? Donc retrait car bug lors de l'ouverture avec la nouvelle libcitygml.
		/*if(rootLevel)
		{
			m_currentAppearence = xmlNewChild(res, NULL, BAD_CAST "app:appearance", NULL);
		}*/

		xmlNodePtr node;
		if(res && obj.getGeometries().size() > 0) //// !! ATTENTION !! : Ne fonctionne que si toutes les géométries ont le même LOD. A modifier pour la gestion des différents Lods.
		{
			switch(obj.getType())
			{
				case COT_TINRelief:
					{
						xmlNodePtr node1 = xmlNewChild(res, NULL, BAD_CAST( type + "lod").c_str(), BAD_CAST std::to_string(obj.getGeometry(0)->getLOD()).c_str());
						xmlNodePtr node2 = xmlNewChild(res, NULL, BAD_CAST( type + "tin").c_str(), NULL);
						xmlNodePtr node3 = xmlNewChild(node2, NULL, BAD_CAST "gml:TriangulatedSurface", NULL);
						std::string id = obj.getId() + "_POLY";
						xmlNewProp(node3, BAD_CAST "gml:id", BAD_CAST id.c_str());
						node = xmlNewChild(node3, NULL, BAD_CAST "gml:trianglePatches", NULL);
						break;
					}
				case COT_WaterBody:
					{
						xmlNodePtr node1 = xmlNewChild(res, NULL, BAD_CAST(type + "boundedBy").c_str(), NULL);
						xmlNodePtr nodeSurface = xmlNewChild(node1, NULL, BAD_CAST(type + "WaterSurface").c_str(), NULL);
						std::string id;
						std::stringstream ss; ss << "PtrId_" << nodeSurface; id = ss.str();
						xmlNewProp(nodeSurface, BAD_CAST "gml:id", BAD_CAST id.c_str());
						xmlNodePtr nodeLodSfc = xmlNewChild(nodeSurface, NULL, BAD_CAST (type + std::string("lod")+std::to_string(obj.getGeometry(0)->getLOD())+"Surface").c_str(), NULL);
						node = xmlNewChild(nodeLodSfc, NULL, BAD_CAST "gml:CompositeSurface", NULL);
						break;
					}
				default:
					{
						xmlNodePtr node1 = xmlNewChild(res, NULL, BAD_CAST (type + std::string("lod")+std::to_string(obj.getGeometry(0)->getLOD())+"MultiSurface").c_str(), NULL);
						node = xmlNewChild(node1, NULL, BAD_CAST "gml:MultiSurface", NULL);
						xmlNewProp(node, BAD_CAST "srsDimension", BAD_CAST "3");
					}
			}
		}

		for(const auto& geom : obj.getGeometries())
		{
			//std::cout << "Geometry" << std::endl;

			if(res)
				exportGeometryXml(*geom, node);
			else exportGeometryXml(*geom, parent);
		}
		if (obj._isXlink!=xLinkState::LINKED)
		for(const auto& child : obj.getChildren()) //Parcourt les WallSurface, RoofSurface par exemple d'un bâtiment.
		{
			if(res) exportCityObjetXml(*child, res);
			else exportCityObjetXml(*child, parent);
		}

		// clear apperance node for current node
		if(rootLevel)
		{
			m_currentAppearence = nullptr;
		}

		return res;
	}
	////////////////////////////////////////////////////////////////////////////////
	std::string getWrapMode(Texture::WrapMode mode)
	{
		switch(mode)
		{
		case Texture::WM_WRAP:
			return "wrap";
			break;
		case Texture::WM_MIRROR:
			return "mirror";
			break;
		case Texture::WM_CLAMP:
			return "clamp";
			break;
		case Texture::WM_BORDER:
			return "border";
			break;
		case Texture::WM_NONE:
			return "none";
			break;
		}
		return "none";
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportPolygonAppearanceXml(const citygml::Polygon& poly, xmlNodePtr parent)
	{
		if(!parent || !poly.getTexture()) return nullptr;

		std::string buf;

		xmlNodePtr root = xmlNewChild(parent, NULL, BAD_CAST "app:Appearance", NULL);
		xmlNewChild(root, NULL, BAD_CAST "app:theme", BAD_CAST "texturation");
		xmlNodePtr srf = xmlNewChild(root, NULL, BAD_CAST "app:surfaceDataMember", NULL);
		xmlNodePtr tex = xmlNewChild(srf, NULL, BAD_CAST "app:ParameterizedTexture", NULL);
		xmlNewChild(tex, NULL, BAD_CAST "app:imageURI", BAD_CAST poly.getTexture()->getUrl().c_str());
		//xmlNewChild(tex, NULL, BAD_CAST "app:mimeType", BAD_CAST "image/tiff");
		xmlNewChild(tex, NULL, BAD_CAST "app:wrapMode", BAD_CAST getWrapMode(poly.getTexture()->getWrapMode()).c_str());
		xmlNodePtr target = xmlNewChild(tex, NULL, BAD_CAST "app:target", NULL);
		buf = "#"+poly.getId();
		xmlNewProp(target, BAD_CAST "uri", BAD_CAST buf.c_str());
		xmlNodePtr tcl = xmlNewChild(target, NULL, BAD_CAST "app:TexCoordList", NULL);
		buf="";
		for(const TVec2f& coord : poly.getTexCoords())
		{
			buf += std::to_string(coord.x) + ' ' + std::to_string(coord.y) + ' ';
		}
		xmlNodePtr tc = xmlNewChild(tcl, NULL, BAD_CAST "app:textureCoordinates", BAD_CAST buf.c_str());
		buf = "#"+poly.getExteriorRing()->getId();
		xmlNewProp(tc, BAD_CAST "ring", BAD_CAST buf.c_str());

		return root;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportCityModelXml(const citygml::CityModel& model)
	{
		//xmlNodePtr root = xmlNewNode(NULL, BAD_CAST "core:CityModel");
		xmlNodePtr root = xmlNewNode(NULL, BAD_CAST "CityModel");
		
		xmlNewProp(root, BAD_CAST "xmlns", BAD_CAST "http://www.opengis.net/citygml/1.0");
        xmlNewProp(root, BAD_CAST "xmlns:app", BAD_CAST "http://www.opengis.net/citygml/appearance/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:bldg", BAD_CAST "http://www.opengis.net/citygml/building/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:core", BAD_CAST "http://www.opengis.net/citygml/base/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:dem", BAD_CAST "http://www.opengis.net/citygml/relief/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:wtr", BAD_CAST "http://www.opengis.net/citygml/waterbody/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:tran", BAD_CAST "http://schemas.opengis.net/citygml/transportation/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:gen", BAD_CAST "http://www.opengis.net/citygml/generics/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:gml", BAD_CAST "http://www.opengis.net/gml");
		xmlNewProp(root, BAD_CAST "xmlns:tex", BAD_CAST "http://www.opengis.net/citygml/textures/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:xsi", BAD_CAST "http://www.w3.org/2001/XMLSchema-instance");
		xmlNewProp(root, BAD_CAST "xsi:schemaLocation", BAD_CAST "http://www.opengis.net/citygml/building/1.0 http://schemas.opengis.net/citygml/building/1.0/building.xsd http://www.opengis.net/citygml/appearance/1.0 http://schemas.opengis.net/citygml/appearance/1.0/appearance.xsd http://www.opengis.net/citygml/relief/1.0 http://schemas.opengis.net/citygml/relief/1.0/relief.xsd");


		// write envelope (bouned by)
        xmlNodePtr nodeEnv = xmlNewChild(root, NULL, BAD_CAST "gml:boundedBy", NULL);
		exportEnvelopeXml(model.getEnvelope(), nodeEnv);

		// do objects
		for(const citygml::CityObject* obj : model.getCityObjectsRoots())
		{
			//xmlNodePtr node = xmlNewChild(root, NULL, BAD_CAST "core:cityObjectMember", NULL);
			xmlNodePtr node = xmlNewChild(root, NULL, BAD_CAST "cityObjectMember", NULL);
			exportCityObjetXml(*obj, node, true);
		}

		return root;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportListTextures(xmlNodePtr root, std::vector<TextureCityGML*>* ListTextures)
	{
		xmlNodePtr AppMember = xmlNewChild(root, NULL, BAD_CAST "app:appearanceMember", NULL);
		xmlNodePtr App = xmlNewChild(AppMember, NULL, BAD_CAST "app:Appearance", NULL);
		for(TextureCityGML* Tex : *ListTextures)
		{
			xmlNewChild(App, NULL, BAD_CAST "app:theme", BAD_CAST "texturation");
			xmlNodePtr srf = xmlNewChild(App, NULL, BAD_CAST "app:surfaceDataMember", NULL);
			xmlNodePtr tex = xmlNewChild(srf, NULL, BAD_CAST "app:ParameterizedTexture", NULL);
			xmlNewChild(tex, NULL, BAD_CAST "app:imageURI", BAD_CAST Tex->Url.c_str());
			xmlNewChild(tex, NULL, BAD_CAST "app:wrapMode", BAD_CAST getWrapMode(Tex->Wrap).c_str());
			for(TexturePolygonCityGML Poly : Tex->ListPolygons)
			{
				xmlNodePtr target = xmlNewChild(tex, NULL, BAD_CAST "app:target", NULL);
				xmlNewProp(target, BAD_CAST "uri", BAD_CAST ("#" + Poly.Id).c_str());
				xmlNodePtr tcl = xmlNewChild(target, NULL, BAD_CAST "app:TexCoordList", NULL);
				std::string buf = "";
				for(const TVec2f& coord : Poly.TexUV)
				{
					buf += std::to_string(coord.x) + ' ' + std::to_string(coord.y) + ' ';
				}
				xmlNodePtr tc = xmlNewChild(tcl, NULL, BAD_CAST "app:textureCoordinates", BAD_CAST buf.c_str());
				buf = "#"+Poly.IdRing;
				xmlNewProp(tc, BAD_CAST "ring", BAD_CAST buf.c_str());
			}
		}

		return root;
	}
	////////////////////////////////////////////////////////////////////////////////
	xmlNodePtr ExporterCityGML::exportCityObjectModelXml(const std::vector<const CityObject*>& objs)
	{
		//xmlNodePtr root = xmlNewNode(NULL, BAD_CAST "core:CityModel");
        xmlNodePtr root = xmlNewNode(NULL, BAD_CAST "CityModel");

		xmlNewProp(root, BAD_CAST "xmlns", BAD_CAST "http://www.opengis.net/citygml/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:app", BAD_CAST "http://www.opengis.net/citygml/appearance/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:bldg", BAD_CAST "http://www.opengis.net/citygml/building/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:core", BAD_CAST "http://www.opengis.net/citygml/base/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:dem", BAD_CAST "http://www.opengis.net/citygml/relief/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:wtr", BAD_CAST "http://www.opengis.net/citygml/waterbody/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:tran", BAD_CAST "http://schemas.opengis.net/citygml/transportation/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:gen", BAD_CAST "http://www.opengis.net/citygml/generics/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:gml", BAD_CAST "http://www.opengis.net/gml");
		xmlNewProp(root, BAD_CAST "xmlns:tex", BAD_CAST "http://www.opengis.net/citygml/textures/1.0");
		xmlNewProp(root, BAD_CAST "xmlns:xsi", BAD_CAST "http://www.w3.org/2001/XMLSchema-instance");
		xmlNewProp(root, BAD_CAST "xsi:schemaLocation", BAD_CAST "http://www.opengis.net/citygml/building/1.0 http://schemas.opengis.net/citygml/building/1.0/building.xsd http://www.opengis.net/citygml/appearance/1.0 http://schemas.opengis.net/citygml/appearance/1.0/appearance.xsd http://www.opengis.net/citygml/relief/1.0 http://schemas.opengis.net/citygml/relief/1.0/relief.xsd");

		// write envelope (bouned by)
		xmlNodePtr node = xmlNewChild(root, NULL, BAD_CAST "gml:boundedBy", NULL);

		Envelope env;
		for(const CityObject* obj : objs) // compute enveloppe
		{
			env.merge(obj->getEnvelope());
		}
		exportEnvelopeXml(env, node);

		// do objects
		for(const CityObject* obj : objs)
		{
			//xmlNodePtr node = xmlNewChild(root, NULL, BAD_CAST "core:cityObjectMember", NULL);
			xmlNodePtr node = xmlNewChild(root, NULL, BAD_CAST "cityObjectMember", NULL);

			exportCityObjetXml(*obj, node, true);
		}

		return root;
	}
	////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
