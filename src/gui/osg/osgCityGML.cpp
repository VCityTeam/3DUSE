// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
// OSG plugin for reading OGC CityGML v0.3 - v1.0 format using libcitygml
// http://code.google.com/p/libcitygml
// Copyright(c) 2010 Joachim Pouderoux, BRGM
////////////////////////////////////////////////////////////////////////////////
#include "osgCityGML.hpp"

#include <osg/Array>
#include <osg/Node>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/TexGen>
#include <osg/TexMat>
#include <osg/Depth>
#include <osg/LightModel>
#include <osg/ValueObject>

#include <osgText/Font>
#include <osgText/Text>

#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>

#include "citygml.hpp"
#include "gui/osg/osgTools.hpp"

#include <algorithm>
#include <cctype> 

#include "core/application.hpp"
////////////////////////////////////////////////////////////////////////////////
ReaderOsgCityGML::ReaderOsgCityGML(const std::string& filepath)
    : m_settings()
{
    m_settings.m_filepath = filepath;
}
////////////////////////////////////////////////////////////////////////////////
ReaderOsgCityGML::Settings::Settings()
    : _printNames( false ), _first(true), _useMaxLODOnly(false), _recursive(false), m_useTextures(true), _origin( 0.f, 0.f, 0.f )
{
}
////////////////////////////////////////////////////////////////////////////////
// Read CityGML file using libcitygml and generate the OSG scenegraph
osg::ref_ptr<osg::Node> ReaderOsgCityGML::readNode(const citygml::CityObject* /*citygml*/)
{
    osg::ref_ptr<osg::Node> res;// = readCity(citygml, settings);

    return res;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Node> ReaderOsgCityGML::readCity(const citygml::CityModel* citygml)
{
    if(!citygml) return NULL;

    osg::notify(osg::NOTICE) << citygml->size() << " city objects read." << std::endl;
	
	osg::notify(osg::NOTICE) << "Creation of the OSG city objects' geometry..." << std::endl;
	
	// Apply translation
    const TVec3d& t = citygml->getTranslationParameters();
	
	osg::MatrixTransform *root = new osg::MatrixTransform();

    root->setName(citygml->getId());

    if(!m_settings._recursive)
    {
        const citygml::CityObjectsMap& cityObjectsMap = citygml->getCityObjectsMap();
        citygml::CityObjectsMap::const_iterator it = cityObjectsMap.begin();

        for(; it != cityObjectsMap.end(); ++it)
        {
            const citygml::CityObjects& v = it->second;

            osg::notify(osg::NOTICE) << " Creation of " << v.size() << " " << citygml::getCityObjectsClassName( it->first ) << ( ( v.size() > 1 ) ? "s" : "" ) << "..." << std::endl;

            osg::Group* grp = new osg::Group;
            grp->setName( citygml::getCityObjectsClassName(it->first ));
            root->addChild(grp);

            for(unsigned int i = 0; i < v.size(); ++i)
            {
                createCityObject(v[i]);
            }
        }
    }
    else
    {
        const citygml::CityObjects& roots = citygml->getCityObjectsRoots();
        for(unsigned int i = 0; i < roots.size(); ++i)
        {
            createCityObject(roots[i]);
        }
    }
	
    root->setMatrix(osg::Matrixd::translate(t.x + m_settings._origin.x(), t.y + m_settings._origin.y(), t.z + m_settings._origin.z()));

	osg::notify(osg::NOTICE) << "Done." << std::endl;

	return root;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Group> ReaderOsgCityGML::createCityObject(const citygml::CityObject* object, unsigned int minimumLODToConsider)
{
	// Skip objects without geometry
    if(!object) return nullptr;

    TVec3d offset_ = vcity::app().getSettings().getDataProfile().m_offset;
    osg::Vec3d offset(offset_.x, offset_.y, offset_.z);

    //osg::ref_ptr<osg::Group> grp = new osg::Group;
    osg::ref_ptr<osg::PositionAttitudeTransform> grp = new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::Geode> geodeLOD0 = new osg::Geode;
    osg::ref_ptr<osg::Geode> geodeLOD1 = new osg::Geode;
    osg::ref_ptr<osg::Geode> geodeLOD2 = new osg::Geode;
    osg::ref_ptr<osg::Geode> geodeLOD3 = new osg::Geode;
    osg::ref_ptr<osg::Geode> geodeLOD4 = new osg::Geode;
    grp->addChild(geodeLOD0);
    grp->addChild(geodeLOD1);
    grp->addChild(geodeLOD2);
    grp->addChild(geodeLOD3);
    grp->addChild(geodeLOD4);
    geodeLOD0->setName(object->getId());
    geodeLOD1->setName(object->getId());
    geodeLOD2->setName(object->getId());
    geodeLOD3->setName(object->getId());
    geodeLOD4->setName(object->getId());
    grp->setName(object->getId());

    //std::cout << "createCityObject : " << object->getId() << std::endl;

    /*if(settings._recursive)
    {
        osg::Group* grp = new osg::Group;
        grp->setName(object->getId());
        grp->addChild(geode);
        parent->addChild(grp);
    }
    else
    {
        parent->addChild(geode);
    }*/

    unsigned int highestLOD = ReaderOsgCityGML::getHighestLodForObject(object);

    for(unsigned int i = 0; i < object->size(); i++)
	{
		const citygml::Geometry& geometry = *object->getGeometry( i );

        const unsigned int currentLOD = geometry.getLOD();

        if(m_settings._useMaxLODOnly && (currentLOD < highestLOD || currentLOD < minimumLODToConsider ))
        {
            continue;
        }

        for(unsigned int j = 0; j < geometry.size(); j++)
		{
			const citygml::Polygon* p = geometry[j];
            
            if(p->getIndices().size() == 0) continue;

			// Geometry management

			osg::Geometry* geom = new osg::Geometry;

			// Vertices
			osg::Vec3Array* vertices = new osg::Vec3Array;
			const std::vector<TVec3d>& vert = p->getVertices();
			vertices->reserve( vert.size() );
            for(unsigned int k = 0; k < vert.size(); k++)
			{
				osg::Vec3d pt( vert[k].x, vert[k].y, vert[k].z );
                if ( m_settings._first )
				{
                    m_settings._origin.set( pt );
                    m_settings._first = false;
				}
                //vertices->push_back( pt - m_settings._origin );
                vertices->push_back(pt - offset);
			}

			geom->setVertexArray( vertices );

			// Indices
			osg::DrawElementsUInt* indices = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 );
            const std::vector<unsigned int>& ind = p->getIndices();
			indices->reserve( ind.size() );
            for(unsigned int i = 0 ; i < ind.size() / 3; i++)
			{
				indices->push_back( ind[ i * 3 + 0 ] );
				indices->push_back( ind[ i * 3 + 1 ] );
				indices->push_back( ind[ i * 3 + 2 ] );	
			}

			geom->addPrimitiveSet( indices );

            // Normals
			osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
			const std::vector<TVec3f>& norm = p->getNormals();
			normals->reserve( norm.size() );
            for(unsigned int k = 0; k < norm.size(); k++)
				normals->push_back( osg::Vec3( norm[k].x, norm[k].y, norm[k].z ) );

			geom->setNormalArray( normals.get() );
			geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

            // Material management (Color management)
            osg::ref_ptr<osg::StateSet> stateset = geom->getOrCreateStateSet(); //Get the stateset from the geometry

            //Create Material and setColorMode to OFF to be able to choose ambient, diffuse and specular color (emission and shininess can also be changed).
            osg::Material* material = new osg::Material;
            material->setColorMode( osg::Material::OFF );

            //Default Grey color
            osg::Vec4 ambientColor = osg::Vec4(0.7f,0.7f,0.7f,1.f);
            osg::Vec4 diffuseColor = osg::Vec4(0.5f,0.5f,0.5f,1.f);
            osg::Vec4 specularColor = osg::Vec4(0.2f,0.2f,0.2f,1.f);

            //Change color depending on object Type
            if(geometry.getType() == citygml::GT_Roof ||
                    object->getType() == citygml::COT_RoofSurface) //Roofs
            {
                //Brick
                ambientColor = osg::Vec4(0.8f,0.25f,0.25f,1.f);
                diffuseColor = osg::Vec4(0.30f,0.30f,0.30f,1.0f);
                specularColor = osg::Vec4(0.2f,0.2f,0.2f,1.f);
            }
            else if(object->getType() == citygml::COT_TINRelief) //Land
            {
                //Brown
                ambientColor = osg::Vec4(0.74f,0.65f,0.56f,1.f);
                diffuseColor = osg::Vec4(0.35f,0.35f,0.35f,1.f);
            }
			else if(object->getType() == citygml::COT_WaterBody )
            {
                //Blue
                ambientColor = osg::Vec4(0.12f,0.49f,0.79f,1.f);
            }
            else if(object->getType() == citygml::COT_SolitaryVegetationObject || object->getType() == citygml::COT_PlantCover || object->getType() == citygml::COT_Square)
            {
                //Green
                ambientColor = osg::Vec4(0.22f,0.54f,0.13f,1.f);
                diffuseColor = osg::Vec4(0.3f,0.3f,0.3f,1.f);
            }

            material->setAmbient( osg::Material::FRONT_AND_BACK, ambientColor );
            material->setDiffuse( osg::Material::FRONT_AND_BACK, diffuseColor );
            material->setSpecular( osg::Material::FRONT_AND_BACK, specularColor );

            stateset->setAttributeAndModes( material, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
            stateset->setMode( GL_LIGHTING, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );


            if ( m_settings.m_useTextures)
            {
                const citygml::Appearance *mat = p->getAppearance();

                if ( const citygml::Texture* t = dynamic_cast<const citygml::Texture*>( mat ) )
                {
                    const citygml::TexCoords& texCoords = p->getTexCoords();

                    if ( texCoords.size() > 0 )
                    {
                        osg::Texture2D* texture = nullptr;

                        if(m_settings._textureMap.find(t->getUrl()) == m_settings._textureMap.end())
                        {
                            // Load a new texture
                            //osg::notify(osg::NOTICE) << "  Loading texture " << t->getUrl() << " for polygon " << p->getId() << "..." << std::endl;

                            if(osg::Image* image = osgDB::readImageFile(m_settings.m_filepath+"/"+t->getUrl()))
                            {
                                //osg::notify(osg::NOTICE) << "  Info: Texture " << m_settings.m_filepath+"/"+t->getUrl() << " loaded." << std::endl;
                                //std::cout << "  Loading texture " << t->getUrl() << " for polygon " << p->getId() << "..." << std::endl;
                                texture = new osg::Texture2D;
                                texture->setImage( image );
                                texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
                                texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
                                texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
                                texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
                                texture->setWrap( osg::Texture::WRAP_R, osg::Texture::REPEAT );
                            }
                            else
                                osg::notify(osg::NOTICE) << "  Warning: Texture " << t->getUrl() << " not found!" << std::endl;

                            m_settings._textureMap[ t->getUrl() ] = texture;
                        }
                        else
                            texture = m_settings._textureMap[ t->getUrl() ];

                        if(texture)
                        {
                            osg::ref_ptr<osg::Vec2Array> tex = new osg::Vec2Array;

                            tex->reserve( texCoords.size() );

                            // georeferencedtexture special case : need to divide texccords by image size
                            if(dynamic_cast<const citygml::GeoreferencedTexture*>( mat ))
                            {
                                float w = texture->getImage()->s();
                                float h = texture->getImage()->t();
                                /*citygml::TexCoords& tc = p->getTexCoords(); // fail, cityobject is const...
                                for ( unsigned int k = 0; k < tc.size(); k++ )
                                {
                                    tc[k].x /= w;
                                    tc[k].x /= h;
                                }*/
                                for ( unsigned int k = 0; k < texCoords.size(); k++ )
                                {
                                    tex->push_back( osg::Vec2( texCoords[k].x/w, texCoords[k].y/h ) );
                                }
                            }
                            else
                            {
                                for ( unsigned int k = 0; k < texCoords.size(); k++ )
                                {
                                    tex->push_back( osg::Vec2( texCoords[k].x, texCoords[k].y ) );
                                }
                            }

                            geom->setTexCoordArray( 0, tex );

                            stateset->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON );
                        }
                    }
                    else
                    {
                        osg::notify(osg::NOTICE) << "  Warning: Texture coordinates not found for poly " << p->getId() << std::endl;
                    }
                }
            }
#if 0
            // Set lighting model to two sided
			osg::ref_ptr< osg::LightModel > lightModel = new osg::LightModel;
			lightModel->setTwoSided( true );
			stateset->setAttributeAndModes( lightModel.get(), osg::StateAttribute::ON );
#endif
            // That's it!
            grp->getChild(geometry.getLOD())->asGeode()->addDrawable(geom);
            //geode->addDrawable( geom );
		}
	}

    if ( m_settings._printNames )
	{
		// Print the city object name on top of it
        geodeLOD0->getBoundingBox().center();
		osg::ref_ptr<osgText::Text> text = new osgText::Text;
		text->setFont( "arial.ttf" );
		text->setCharacterSize( 2 );
		text->setBackdropType( osgText::Text::OUTLINE );
		text->setFontResolution( 64, 64 );
		text->setText( object->getId(), osgText::String::ENCODING_UTF8 );
		text->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
		text->setAxisAlignment( osgText::TextBase::SCREEN );
		text->setAlignment( osgText::TextBase::CENTER_BOTTOM );
        text->setPosition( geodeLOD0->getBoundingBox().center() + osg::Vec3( 0, 0, geodeLOD0->getBoundingBox().radius() ) );
		text->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF );
        geodeLOD0->addDrawable( text.get() );
	}

	// Manage transparency for windows
    if(object->getType() == citygml::COT_Window)
	{
        osg::StateSet* geodeSS(geodeLOD0->getOrCreateStateSet());

        osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc(osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA,osg::BlendFunc::CONSTANT_ALPHA);
		geodeSS->setAttributeAndModes( blendFunc.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON );

        osg::ref_ptr<osg::BlendColor> blendColor = new osg::BlendColor(osg::Vec4( 1., 1., 1., object->getDefaultColor().a ));
		geodeSS->setAttributeAndModes( blendColor.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON );

        osg::ref_ptr<osg::Depth> depth = new osg::Depth;
        depth->setWriteMask( false );
        geodeSS->setAttributeAndModes( depth.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON );
   
		geodeSS->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}

    /*if(settings._recursive)
    {
        for(unsigned int i = 0; i < object->getChildCount(); ++i)
            createCityObject( object->getChild(i), settings, grp, highestLOD);
    }*/

    // add bbox
    if(object->getType() == citygml::COT_Building)
    {
        //object->computeEnvelope();
        const citygml::Envelope& env = object->getEnvelope();
        if ( (env.getUpperBound().x != 0.0 && env.getUpperBound().y != 0.0 && env.getUpperBound().z != 0.0) ||
           (env.getLowerBound().x != 0.0 && env.getLowerBound().y != 0.0 && env.getLowerBound().z != 0.0) )
        {
            //std::cout << object->getId() << " : " << env << std::endl;
            TVec3d lb_ = env.getLowerBound()-offset_;
            osg::Vec3 lb(lb_.x, lb_.y, lb_.z);
            TVec3d ub_ = env.getUpperBound()-offset_;
            osg::Vec3 ub(ub_.x, ub_.y, ub_.z);
            //std::cout << lb.x() << ", " << lb.y() << ", " << lb.z() << std::endl;
            //std::cout << ub.x() << ", " << ub.y() << ", " << ub.z() << std::endl;
            osg::ref_ptr<osg::Geode> bbox = osgTools::buildBBox(lb, ub);
            //geode->addDrawable(bbox->getDrawable(0));
        }
    }

    // temporal
    std::string strAttr = object->getAttribute("yearOfConstruction");
    int yearOfConstruction = (strAttr.empty()?-4000:std::stoi(strAttr));
    strAttr = object->getAttribute("yearOfDemolition");
    int yearOfDemolition = (strAttr.empty()?-4000:std::stoi(strAttr));

    if(yearOfConstruction != -4000)
    {
        grp->setUserValue("yearOfConstruction", yearOfConstruction);
    }
    if(yearOfDemolition != -4000)
    {
        grp->setUserValue("yearOfDemolition", yearOfDemolition);
    }
	strAttr = object->getAttribute("creationDate");
	if(!strAttr.empty()) grp->setUserValue("creationDate", strAttr);
	strAttr = object->getAttribute("terminationDate");
	if(!strAttr.empty()) grp->setUserValue("terminationDate", strAttr);	

    //std::cout << "build osg geom ok" << std::endl;
    return grp;
}
////////////////////////////////////////////////////////////////////////////////
unsigned int ReaderOsgCityGML::getHighestLodForObject( const citygml::CityObject* object)
{
    unsigned int highestLOD = 0;
    // first find out highest LOD for this object
    for(unsigned int i = 0; i < object->size(); i++)
    {
        const citygml::Geometry &geometry = *object->getGeometry(i);

        if(geometry.getLOD() > highestLOD)
        {
            highestLOD = geometry.getLOD();
        }
    }

    /*if(settings._recursive)
    {
        //check for the highest LODs of Children
        for(unsigned int i = 0; i < object->getChildCount(); ++i)
        {
            unsigned int tempHighestLOD = ReaderOsgCityGML::getHighestLodForObject(object->getChild(i));
            if(tempHighestLOD > highestLOD)
            {
                tempHighestLOD = highestLOD;
            }
        }
    }*/

    return highestLOD;
}
////////////////////////////////////////////////////////////////////////////////
