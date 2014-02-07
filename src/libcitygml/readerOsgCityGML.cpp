////////////////////////////////////////////////////////////////////////////////
// OSG plugin for reading OGC CityGML v0.3 - v1.0 format using libcitygml
// http://code.google.com/p/libcitygml
// Copyright(c) 2010 Joachim Pouderoux, BRGM
////////////////////////////////////////////////////////////////////////////////
#include "readerOsgCityGML.hpp"

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

    //osg::ref_ptr<osg::Group> grp = new osg::Group;
    osg::ref_ptr<osg::PositionAttitudeTransform> grp = new osg::PositionAttitudeTransform;
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    grp->addChild(geode);
    geode->setName(object->getId());
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

	// Get the default color for the whole city object
	osg::ref_ptr<osg::Vec4Array> shared_colors = new osg::Vec4Array;
	shared_colors->push_back( osg::Vec4( object->getDefaultColor().r, object->getDefaultColor().g, object->getDefaultColor().b, object->getDefaultColor().a ) );

	osg::ref_ptr<osg::Vec4Array> roof_color = new osg::Vec4Array;
	roof_color->push_back( osg::Vec4( 0.9f, 0.1f, 0.1f, 1.0f ) );

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
                vertices->push_back(pt - osg::Vec3d(643000.0, 6857000.0, 0.0));
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

			// Material management

			osg::ref_ptr<osg::StateSet> stateset = geom->getOrCreateStateSet();

			const citygml::Appearance *mat = p->getAppearance();

			bool colorset = false;

            if(mat)
			{
				shared_colors->clear();
				shared_colors->push_back( osg::Vec4( 1.f, 1.f, 1.f, 1.f ) );

				if ( const citygml::Material* m = dynamic_cast<const citygml::Material*>( mat ) )
				{
#define TOVEC4(_t_) osg::Vec4( _t_.r, _t_.g, _t_.b, _t_.a ) 
                    //osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
                    //TVec4f diffuse( m->getDiffuse(), 0.f );
                    //TVec4f emissive( m->getEmissive(), 0.f );
                    //TVec4f specular( m->getSpecular(), 0.f );
                    //float ambient = m->getAmbientIntensity();

					osg::Material* material = new osg::Material;
					material->setColorMode( osg::Material::OFF );
                    //material->setDiffuse( osg::Material::FRONT_AND_BACK, TOVEC4( diffuse ) );
                    //material->setSpecular( osg::Material::FRONT_AND_BACK, TOVEC4( specular ) );
                    //material->setEmission( osg::Material::FRONT_AND_BACK, TOVEC4( emissive ) );
                    //material->setShininess( osg::Material::FRONT_AND_BACK, m->getShininess() );
                    //material->setAmbient( osg::Material::FRONT_AND_BACK, osg::Vec4( ambient, ambient, ambient, 1.0 ) );
                    material->setDiffuse( osg::Material::FRONT_AND_BACK, osg::Vec4(1,1,1,1) );
                    material->setSpecular( osg::Material::FRONT_AND_BACK, osg::Vec4(1,1,1,1) );
                    material->setEmission( osg::Material::FRONT_AND_BACK, osg::Vec4(1,1,1,1) );
                    material->setAmbient( osg::Material::FRONT_AND_BACK, osg::Vec4(1,1,1,1) );
                    //material->setTransparency( osg::Material::FRONT_AND_BACK, m->getTransparency() );
					stateset->setAttributeAndModes( material, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );
					stateset->setMode( GL_LIGHTING, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON );

					colorset = true;
				}
                else if ( m_settings.m_useTextures)
                {
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

                                colorset = true;
                            }
                        }
                        else
                        {
                            osg::notify(osg::NOTICE) << "  Warning: Texture coordinates not found for poly " << p->getId() << std::endl;
                        }
                    }
                }
			}

			// Color management

			geom->setColorArray( ( !colorset && geometry.getType() == citygml::GT_Roof ) ? roof_color.get() : shared_colors.get() );

			geom->setColorBinding( osg::Geometry::BIND_OVERALL );
#if 0
			// Set lighting model to two sided
			osg::ref_ptr< osg::LightModel > lightModel = new osg::LightModel;
			lightModel->setTwoSided( true );
			stateset->setAttributeAndModes( lightModel.get(), osg::StateAttribute::ON );
#endif
			// That's it!
			geode->addDrawable( geom );			
		}
	}

    if ( m_settings._printNames )
	{
		// Print the city object name on top of it
		geode->getBoundingBox().center();
		osg::ref_ptr<osgText::Text> text = new osgText::Text;
		text->setFont( "arial.ttf" );
		text->setCharacterSize( 2 );
		text->setBackdropType( osgText::Text::OUTLINE );
		text->setFontResolution( 64, 64 );
		text->setText( object->getId(), osgText::String::ENCODING_UTF8 );
		text->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
		text->setAxisAlignment( osgText::TextBase::SCREEN );
		text->setAlignment( osgText::TextBase::CENTER_BOTTOM );
		text->setPosition( geode->getBoundingBox().center() + osg::Vec3( 0, 0, geode->getBoundingBox().radius() ) );
		text->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF );
		geode->addDrawable( text.get() );
	}

	// Manage transparency for windows
    if(object->getType() == citygml::COT_Window)
	{
        osg::StateSet* geodeSS(geode->getOrCreateStateSet());

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
    const citygml::Envelope& env = object->getEnvelope();
    if(object->getType() == citygml::COT_Building && env.getLowerBound().x != 0 && env.getLowerBound().y != 0 && env.getLowerBound().z != 0 &&
                                                     env.getUpperBound().x != 0 && env.getUpperBound().y != 0 && env.getUpperBound().z != 0)
    //std::cout << "bbox : " << env << std::endl;
    //if(object->getType() == citygml::COT_Building)
    {
        //object->computeEnvelope();
        //const citygml::Envelope& env = object->getEnvelope();
        TVec3d lb_ = env.getLowerBound()-TVec3d(643000.0, 6861500.0, 0.0);
        osg::Vec3 lb(lb_.x, lb_.y, lb_.z);
        TVec3d ub_ = env.getUpperBound()-TVec3d(643000.0, 6861500.0, 0.0);
        osg::Vec3 ub(ub_.x, ub_.y, ub_.z);
        osg::ref_ptr<osg::Geode> bbox = osgTools::buildBBox(lb, ub);
        geode->addDrawable(bbox->getDrawable(0));

        //std::cout << "bbox ok" << std::endl;
    }

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
