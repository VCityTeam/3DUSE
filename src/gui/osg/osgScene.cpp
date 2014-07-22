////////////////////////////////////////////////////////////////////////////////
#include "osgScene.hpp"
#include <osg/PositionAttitudeTransform>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
//#include <osgShadow/ViewDependentShadowMap>
#include <osgShadow/SoftShadowMap>
#include <osg/Switch>
//#include <osgShadow/ParallelSplitShadowMap>
#include <osg/ValueObject>
#include <osgText/Text>
#include <osgUtil/Optimizer>
#include "gui/applicationGui.hpp"
#include "gui/moc/mainWindow.hpp"
#include "osgCityGML.hpp"
#include "core/dataprofile.hpp"
////////////////////////////////////////////////////////////////////////////////
/** Provide an simple example of customizing the default UserDataContainer.*/
class MyUserDataContainer : public osg::DefaultUserDataContainer
{
    public:
        MyUserDataContainer() {}
        MyUserDataContainer(const MyUserDataContainer& udc, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY):
            DefaultUserDataContainer(udc, copyop) {}

        META_Object(MyNamespace, MyUserDataContainer)

        virtual Object* getUserObject(unsigned int i)
        {
            OSG_NOTICE<<"MyUserDataContainer::getUserObject("<<i<<")"<<std::endl;
            return  osg::DefaultUserDataContainer::getUserObject(i);
        }

        virtual const Object* getUserObject(unsigned int i) const
        {
            OSG_NOTICE<<"MyUserDataContainer::getUserObject("<<i<<") const"<<std::endl;
            return osg::DefaultUserDataContainer::getUserObject(i);
        }

    protected:
        virtual ~MyUserDataContainer() {}
};
////////////////////////////////////////////////////////////////////////////////


class FindNamedNode : public osg::NodeVisitor
{
public:
    FindNamedNode( const std::string& name )
      : osg::NodeVisitor( // Traverse all children.
                osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _name( name )
    {
        setTraversalMask(0xffffffff);
        setNodeMaskOverride(0xffffffff);
    }

    // This method gets called for every node in the scene
    //   graph. Check each node to see if its name matches
    //   out target. If so, save the node's address.
    virtual void apply( osg::Node& node )
    {
        if (node.getName() == _name)
            _node = &node;

        // Keep traversing the rest of the scene graph.
        traverse( node );
    }

    osg::ref_ptr<osg::Node> getNode() { return _node; }

protected:
    std::string _name;
    osg::ref_ptr<osg::Node> _node;
};



OsgScene::OsgScene()
    : osg::Group(), m_shadow(false), m_shadowVec(-1,-1,0.1,0)
{
    init();
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::init()
{
    setName("root");

    osg::ref_ptr<osg::Group> node = NULL;

    // build effectNone node
    m_effectNone = new osg::Group();
    m_effectNone->setName("effect_none");

    // build effectShadow node
    osg::ref_ptr<osgShadow::ShadowedScene> shadowedScene = new osgShadow::ShadowedScene;
    shadowedScene->setName("effect_shadow");
    //osgShadow::ShadowSettings* settings = shadowedScene->getShadowSettings();

    #if 0
    //shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
    //shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
    osg::ref_ptr<osgShadow::ShadowMap> st = new osgShadow::ShadowMap;
    //osg::ref_ptr<osgShadow::ParallelSplitShadowMap> sm = new osgShadow::ParallelSplitShadowMap;
    shadowedScene->setShadowTechnique(st.get());
    int mapres = 2048;
    st->setTextureSize(osg::Vec2s(mapres,mapres));
    //sm->setTextureResolution(mapres);
    #elif 0
    osg::ref_ptr<osgShadow::ViewDependentShadowMap> st = new osgShadow::ViewDependentShadowMap();
    int mapres = 1024;
    settings->setTextureSize(osg::Vec2s(mapres,mapres));
    settings->setMultipleShadowMapHint(osgShadow::ShadowSettings::PARALLEL_SPLIT);
    settings->setNumShadowMapsPerLight(4);
    shadowedScene->setShadowTechnique(st.get());
    #else
    osg::ref_ptr<osgShadow::SoftShadowMap> st = new osgShadow::SoftShadowMap();
    int mapres = 2048;
    st->setTextureSize(osg::Vec2s(mapres,mapres));
    shadowedScene->setShadowTechnique(st.get());
    #endif

    // light, sun
    osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();
    lightSource->setName("lightsource");
    osg::ref_ptr<osg::Light> light = new osg::Light();
    light->setName("light");
    light->setAmbient(osg::Vec4(0.6,0.6,0.6,1.0));
    lightSource->setLight(light);
    light->setPosition(m_shadowVec);
    shadowedScene->addChild(lightSource);
    m_effectNone->addChild(lightSource);
    m_effectShadow = shadowedScene;

    // add first depth node, to handle effect (shadow)
    if(m_shadow)
    {
        addChild(m_effectShadow);
    }
    else
    {
        addChild(m_effectNone);
    }

    // build layers node
    m_layers = new osg::Group();
    m_layers->setName("layers");

    m_effectShadow->addChild(m_layers);
    m_effectNone->addChild(m_layers);

    // build first default layer
    osg::ref_ptr<osg::Group> layer0 = new osg::Group();
    layer0->setName("layer_CityGML");
    m_layers->addChild(layer0);

	// build second default layer
    osg::ref_ptr<osg::Group> layer1 = new osg::Group();
    layer1->setName("layer_Assimp");
    m_layers->addChild(layer1);

	// build third default layer
    osg::ref_ptr<osg::Group> layer2 = new osg::Group();
    layer2->setName("layer_Mnt");
    m_layers->addChild(layer2);

    // build forth default layer
    osg::ref_ptr<osg::Group> layer3 = new osg::Group();
    layer3->setName("layer_Shp");
    m_layers->addChild(layer3);

    //osg::ref_ptr<osg::Geode> geode = buildGrid(osg::Vec3(64300.0, 6861500.0, 0.0), 500.0, 10);
    osg::ref_ptr<osg::Geode> grid = buildGrid(osg::Vec3(0.0, 0.0, 0.0), 500.0, 30);
    //osg::ref_ptr<osg::Geode> grid = buildGrid(osg::Vec3(643000.0, 6857000.0, 0.0), 500.0, 30);
    m_layers->addChild(grid);

    //osg::ref_ptr<osg::Geode> bbox = buildBBox(osg::Vec3(100.0, 100.0, 100.0), osg::Vec3(400.0, 400.0, 400.0));
    //m_layers->addChild(bbox);

	//osg::ref_ptr<osg::Geode> bbox = buildBBox(osg::Vec3(-10.0, -10.0, -10.0), osg::Vec3(10.0, 10.0, 10.0));
    //m_layers->addChild(bbox);
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::addTile(const vcity::URI& uriLayer, const vcity::Tile& tile)
{
    osg::ref_ptr<osg::Node> layer = getNode(uriLayer);
    if(layer)
    {
        osg::ref_ptr<osg::Group> layerGroup = layer->asGroup();
        if(layerGroup)
        {
            osg::ref_ptr<osg::Node> osgTile = buildTile(tile);
            layerGroup->addChild(osgTile);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::setTileName(const vcity::URI& uriTile, const std::string& name)
{
    osg::ref_ptr<osg::Node> tile = getNode(uriTile);
    if(tile)
    {
        tile->setName(name);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::deleteTile(const vcity::URI& uriTile)
{
    osg::ref_ptr<osg::Node> tile = getNode(uriTile);
    if(tile)
    {
        tile->getParent(0)->removeChild(tile);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::addAssimpNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    osg::ref_ptr<osg::Node> layer = getNode(uriLayer);
    if(layer)
    {
        osg::ref_ptr<osg::Group> layerGroup = layer->asGroup();
        if(layerGroup)
			layerGroup->addChild(node);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::setAssimpNodeName(const vcity::URI& uri, const std::string& name)
{
    osg::ref_ptr<osg::Node> assimpNode = getNode(uri);
    if(assimpNode)
    {
        assimpNode->setName(name);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::deleteAssimpNode(const vcity::URI& uri)
{
    osg::ref_ptr<osg::Node> assimpNode = getNode(uri);
    if(assimpNode)
    {
        assimpNode->getParent(0)->removeChild(assimpNode);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::addMntAscNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    osg::ref_ptr<osg::Node> layer = getNode(uriLayer);
    if(layer)
    {
        osg::ref_ptr<osg::Group> layerGroup = layer->asGroup();
        if(layerGroup)
			layerGroup->addChild(node);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::addShpNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node)
{
    osg::ref_ptr<osg::Node> layer = getNode(uriLayer);
    if(layer)
    {
        osg::ref_ptr<osg::Group> layerGroup = layer->asGroup();
        if(layerGroup)
            layerGroup->addChild(node);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::addLayer(const std::string& name)
{
    osg::ref_ptr<osg::Group> layer = new osg::Group();
    layer->setName(name);
    m_layers->addChild(layer);
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::setLayerName(const vcity::URI& uriLayer, const std::string& name)
{
    osg::ref_ptr<osg::Node> layer = getNode(uriLayer);
    if(layer)
    {
        layer->setName(name);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::deleteLayer(const vcity::URI& uriLayer)
{
    osg::ref_ptr<osg::Node> layer = getNode(uriLayer);
    if(layer)
    {
        layer->getParent(0)->removeChild(layer);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::deleteNode(const vcity::URI& uri)
{
    osg::ref_ptr<osg::Node> node = getNode(uri);
    if(node)
    {
        node->getParent(0)->removeChild(node);
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::updateGrid()
{
    for(unsigned int i=0; i<m_layers->getNumChildren(); ++i)
    {
        if(m_layers->getChild(i)->getName() == "grid")
        {
            //m_layers->getChild(i) =
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::setShadow(bool shadow)
{
    if(shadow)
    {
        replaceChild(m_effectNone, m_effectShadow);
    }
    else
    {
        replaceChild(m_effectShadow, m_effectNone);
    }

    m_shadow = shadow;
}
////////////////////////////////////////////////////////////////////////////////
std::map<std::string, osg::ref_ptr<osg::Texture2D> > texManager;
////////////////////////////////////////////////////////////////////////////////
void setTexture(osg::ref_ptr<osg::Node> node, citygml::CityObjectTag* tag, osg::ref_ptr<osg::Texture2D> texture)
{
    osg::ref_ptr<osg::Group> grp = node->asGroup();
    if(grp)
    {
        for(unsigned int i=0; i<grp->getNumChildren(); ++i)
        {
            osg::ref_ptr<osg::Node> child = grp->getChild(i);
            setTexture(child, tag, texture);
        }
    }

    osg::ref_ptr<osg::Geode> geode = node->asGeode();
    if(geode)
    {
        for(unsigned int i=0; i<geode->getNumDrawables(); ++i)
        {
            osg::StateSet* stateset = geode->getDrawable(i)->getOrCreateStateSet();
            //if(texture) stateset->setTextureAttributeAndModes( 0, texture, osg::StateAttribute::ON );
            std::cout << "texture : " << texture << std::endl;
            if(texture) stateset->setTextureAttribute( 0, texture, osg::StateAttribute::ON );
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void setDateRec(const QDateTime& date, osg::ref_ptr<osg::Node> node)
{
    int year = date.date().year();

    osg::ref_ptr<osg::Group> grp = node->asGroup();
    if(grp)
    {
        for(unsigned int i=0; i<grp->getNumChildren(); ++i)
        {
            osg::ref_ptr<osg::Node> child = grp->getChild(i);

            double val;
            bool hasFlag = node->getUserValue("TAGPTR", val);
            if(hasFlag)
            {
                citygml::CityObjectTag* tag;
                memcpy(&tag, &val, sizeof(tag));
                std::string texturePath = tag->getAttribute("texture", date);
                if(texturePath != "none")
                {
                    std::cout << date.toString().toStdString() << " : texture : " << texturePath << std::endl;

                    // check cache
                    osg::ref_ptr<osg::Texture2D> texture = nullptr;
                    std::map<std::string, osg::ref_ptr<osg::Texture2D> >::iterator it = texManager.find(texturePath);
                    if(it!=texManager.end())
                    {
                        texture = it->second;
                    }
                    else
                    {
                        if(osg::Image* image = osgDB::readImageFile(texturePath))
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

                            texManager[texturePath] = texture;
                        }
                        else
                            osg::notify(osg::NOTICE) << "  Warning: Texture " << texturePath << " not found!" << std::endl;
                    }

                    setTexture(node, tag, texture);
                }
            }

            int yearOfConstruction;
            int yearOfDemolition;

            bool a = node->getUserValue("yearOfConstruction", yearOfConstruction);
            bool b = node->getUserValue("yearOfDemolition", yearOfDemolition);

            //std::cout << node->getName() << " : " << a <<  " : yearOfConstruction : " << yearOfConstruction << std::endl;
            //std::cout << node->getName() << " : " << b << " : yearOfDemolition : " << yearOfDemolition << std::endl;

            if(a && b)
            {
                if((yearOfConstruction < year && year < yearOfDemolition) || year == 0)
                {
                    node->setNodeMask(0xffffffff);
                }
                else
                {
                     node->setNodeMask(0);
                }
                //node->setNodeMask(0xffffffff - node->getNodeMask());
            }

            if(date.date().year() == 0)
            {
                node->setNodeMask(0xffffffff);
            }

            setDateRec(date, child);
        }
    }

    osg::ref_ptr<osg::Geode> geode = node->asGeode();
    if(geode)
    {
        int tagged = 0;
        bool c = geode->getUserValue("TAGGED", tagged);
        if(c && tagged)
        {
            geode->setNodeMask(0);
            geode->getParent(0)->setNodeMask(0);

            std::cout << "hide TAGGED default geom : " << geode->getName() << " : " << geode->getNodeMask() << std::endl;
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::setDate(const QDateTime& date)
{
    setDateRec(date, this);
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::reset()
{
    std::vector<osg::Node*> nodes;
    for(unsigned int i=0; i<getNumChildren(); ++i)
    {
        nodes.push_back(getChild(i));
    }
    std::vector<osg::Node*>::iterator it = nodes.begin();
    for(;it != nodes.end(); ++it)
    {
        removeChild(*it);
    }

    init();
}
////////////////////////////////////////////////////////////////////////////////
void forceLODrec(int lod, osg::ref_ptr<osg::Node> node)
{
    osg::ref_ptr<osg::Group> grp = node->asGroup();
    if(grp)
    {
        int count = grp->getNumChildren();
        for(int i=0; i<count; ++i)
        {


            osg::ref_ptr<osg::Node> child = grp->getChild(i);
            forceLODrec(lod, child);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::forceLOD(int lod)
{
    forceLODrec(lod, m_layers);
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::showNode(osg::ref_ptr<osg::Node> node, bool show)
{
    if(node)
    {
        if(show)
        {
            //node->setNodeMask(~0x0);
            node->setNodeMask(0xffffffff);
            if(node->asGroup())
            {
                node->asGroup()->getChild(0)->setNodeMask(0xffffffff);
            }
        }
        else
        {
            node->setNodeMask(0x0);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::showNode(const vcity::URI& uri, bool show)
{
    showNode(getNode(uri), show);
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::centerOn(const vcity::URI& uri)
{
    osg::ref_ptr<osg::Node> node = getNode(uri);
    if(node)
    {
        appGui().getMainWindow()->m_osgView->m_osgView->getCameraManipulator()->setNode(node);
        appGui().getMainWindow()->m_osgView->m_osgView->getCameraManipulator()->computeHomePosition();
        appGui().getMainWindow()->m_osgView->m_osgView->home();
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::dump(std::ostream& out, osg::ref_ptr<osg::Node> node, int depth)
{
    if(node == NULL)
    {
        node = this;
    }

    for(int i=0; i<depth; ++i) out << "  ";
    out << depth << " " << node->getName() << " : " << node << std::endl;

    osg::ref_ptr<osg::Group> grp = node->asGroup();
    if(grp)
    {
        for(unsigned int i=0; i<grp->getNumChildren(); ++i)
        {
            osg::ref_ptr<osg::Node> child = grp->getChild(i);
            dump(out, child, depth+1);
        }
    }

    if(node->asTransform() && node->asTransform()->asPositionAttitudeTransform())
    {
        const osg::Vec3d pos = node->asTransform()->asPositionAttitudeTransform()->getPosition();
        out << "pos : " << pos.x() << ", " << pos.y() << std::endl;
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::optim()
{
    osgUtil::Optimizer optimizer;
    optimizer.optimize(this, osgUtil::Optimizer::ALL_OPTIMIZATIONS);
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::buildCityObject(osg::ref_ptr<osg::Group> nodeOsg, citygml::CityObject* obj, ReaderOsgCityGML& reader, int depth)
{
    osg::ref_ptr<osg::Group> node = reader.createCityObject(obj);
    nodeOsg->addChild(node);

    citygml::CityObjects& cityObjects = obj->getChildren();
    citygml::CityObjects::iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        buildCityObject(node, *it, reader, depth+1);
    }
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Node> OsgScene::buildTile(const vcity::Tile& tile)
{
    osg::ref_ptr<osg::PositionAttitudeTransform> root = new osg::PositionAttitudeTransform();
    root->setName(tile.getName());

    // create osg geometry builder
    size_t pos = tile.getCityGMLfilePath().find_last_of("/\\");
    std::string path = tile.getCityGMLfilePath().substr(0, pos);
    ReaderOsgCityGML readerOsgGml(path);
    readerOsgGml.m_settings.m_useTextures = vcity::app().getSettings().m_loadTextures;

    const citygml::CityObjects& cityObjects = tile.getCityModel()->getCityObjectsRoots();
    citygml::CityObjects::const_iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        buildCityObject(root, *it, readerOsgGml);
    }
    return root;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Node> OsgScene::getNode(const vcity::URI& uri)
{
    osg::ref_ptr<osg::Group> current = m_layers;

    int depth = uri.getDepth()-uri.getCursor();
    //int maxDepth = depth;

    if(depth == 0)
    {
        return nullptr;
    }

    do
    {
        int count = current->getNumChildren();
        for(int i=0; i<count; ++i)
        {
            osg::ref_ptr<osg::Node> child = current->getChild(i);
            //std::cout << child->getName() << " -> " << uri.getNode(maxDepth-depth) << std::endl;
            if(child->getName() == uri.getCurrentNode())
            {
				uri.popFront();
                if(depth == 1)
                {
                    return child;
                }
                else if(!(current = child->asGroup()))
                {
                    return nullptr;
                }
                break;
            }
        }
        --depth;
    } while(depth > 0);

    return nullptr;

    /*FindNamedNode f(uri.getLastNode());
    accept(f);
    return f.getNode();*/
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Node> OsgScene::createInfoBubble(osg::ref_ptr<osg::Node> node)
{
    osg::ref_ptr<osg::Group> grp = node->asGroup();
    if(grp)
    {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->setName("infobubble");

        // Print the city object name on top of it
        geode->getBoundingBox().center();
        osg::ref_ptr<osgText::Text> text = new osgText::Text;
        text->setFont( "arial.ttf" );
        text->setCharacterSize( 24 );
        text->setBackdropType( osgText::Text::OUTLINE );
        text->setFontResolution( 64, 64 );
        text->setText( node->getName(), osgText::String::ENCODING_UTF8 );
        //text->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
        text->setCharacterSizeMode( osgText::TextBase::SCREEN_COORDS );
        text->setAxisAlignment( osgText::TextBase::SCREEN );
        text->setAlignment( osgText::TextBase::CENTER_BOTTOM );
        text->setPosition( node->getBound().center() + osg::Vec3( 0, 0, node->getBound().radius() ) );
        text->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF );
        geode->addDrawable( text.get() );

        grp->addChild(geode);

        return geode;
    }

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> OsgScene::buildGrid(osg::Vec3 origin, float step, int n)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setName("grid");

    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    for(int i=0; i<=n; ++i)
    {
        vertices->push_back(osg::Vec3(origin.x() + i*step, origin.y() + 0, origin.z() + 0));
        vertices->push_back(osg::Vec3(origin.x() + i*step, origin.y() + n*step, origin.z() + 0));

        indices->push_back(4*i);
        indices->push_back(4*i+1);

        vertices->push_back(osg::Vec3(origin.x() + 0, origin.y() + i*step, origin.z() + 0));
        vertices->push_back(osg::Vec3(origin.x() + n*step, origin.y() + i*step, origin.z() + 0));

        indices->push_back(4*i+2);
        indices->push_back(4*i+3);
    }

    for(int x=0; x<n; ++x)
    {
        for(int y=0; y<n; ++y)
        {
            osgText::Text* text = new osgText::Text;
            std::stringstream ss;
            ss << 643000 + origin.x() + x*step << " , " << 6857000 + (int)origin.y() + y*(int)step << "\n" <<
            origin.x() + x*step << " , " << origin.y() + y*step << " , " << origin.z() + 0 << "\nEXPORT_" << 1286+x << "-" << 13714+y;
            text->setText(ss.str());
            text->setColor(osg::Vec4(0,0,0,1));
            text->setPosition(osg::Vec3(origin.x() + x*step, origin.y() + y*step + step*0.2, origin.z() + 0));
            geode->addDrawable(text);
        }
    }

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(indices);
    geode->addDrawable(geom);

    return geode;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> OsgScene::buildBBox(osg::Vec3 lowerBound, osg::Vec3 upperBound)
{
    osg::Vec3 step = upperBound - lowerBound;

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::Geometry* geom = new osg::Geometry;
    osg::Vec3Array* vertices = new osg::Vec3Array;
    osg::DrawElementsUInt* indices = new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);

    for(int x=0; x<=1; ++x)
    {
        for(int y=0; y<=1; ++y)
        {
            for(int z=0; z<=1; ++z)
            {
                vertices->push_back(osg::Vec3(lowerBound.x()+ x*step.x(), lowerBound.y() + y*step.y(), lowerBound.z() + z*step.z()));
                std::cout << lowerBound.x()+ x*step.x() << " " << lowerBound.y() + y*step.y() << " " << lowerBound.z() + z*step.z() << std::endl;
            }
        }
    }

    indices->push_back(0); indices->push_back(1);
    indices->push_back(0); indices->push_back(2);
    indices->push_back(0); indices->push_back(4);
    indices->push_back(3); indices->push_back(1);
    indices->push_back(3); indices->push_back(2);
    indices->push_back(3); indices->push_back(7);
    indices->push_back(5); indices->push_back(1);
    indices->push_back(5); indices->push_back(4);
    indices->push_back(5); indices->push_back(7);
    indices->push_back(6); indices->push_back(2);
    indices->push_back(6); indices->push_back(4);
    indices->push_back(6); indices->push_back(7);

    geom->setVertexArray(vertices);
    geom->addPrimitiveSet(indices);
    geode->addDrawable(geom);

    return geode;
}
////////////////////////////////////////////////////////////////////////////////
