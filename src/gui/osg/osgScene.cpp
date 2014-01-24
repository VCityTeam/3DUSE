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
        _name( name ) {}

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
    : osg::Group(), m_shadow(false), m_shadowVec(-1,-1,1,0)
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
    light->setAmbient(osg::Vec4(1.0,0.0,0.0,1.0));
    lightSource->setLight(light);
    light->setPosition(m_shadowVec);
    shadowedScene->addChild(lightSource);
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

    // build lods node
    /*m_lods = new osg::Group();
    m_lods->setName("lods");
    layer0->addChild(m_lods);

    // build lod0, lod1, lod2, lod3, lod4
    osg::ref_ptr<osg::Switch> lod0 = new osg::Switch();
    lod0->setName("lod0");
    m_lods->addChild(lod0);

    osg::ref_ptr<osg::Switch> lod1 = new osg::Switch();
    lod1->setName("lod1");
    m_lods->addChild(lod1);

    osg::ref_ptr<osg::Switch> lod2 = new osg::Switch();
    lod2->setName("lod2");
    m_lods->addChild(lod2);

    osg::ref_ptr<osg::Switch> lod3 = new osg::Switch();
    lod3->setName("lod3");
    m_lods->addChild(lod3);

    osg::ref_ptr<osg::Switch> lod4 = new osg::Switch();
    lod4->setName("lod4");
    m_lods->addChild(lod4);

    // build tiles node
    m_tilesLod0 = new osg::Group();
    m_tilesLod0->setName("tiles_lod0");
    lod0->addChild(m_tilesLod0);

    m_tilesLod1 = new osg::Group();
    m_tilesLod1->setName("tiles_lod1");
    lod1->addChild(m_tilesLod1);

    m_tilesLod2 = new osg::Group();
    m_tilesLod2->setName("tiles_lod2");
    lod2->addChild(m_tilesLod2);

    m_tilesLod3 = new osg::Group();
    m_tilesLod3->setName("tiles_lod3");
    lod3->addChild(m_tilesLod3);

    m_tilesLod4 = new osg::Group();
    m_tilesLod4->setName("tiles_lod4");
    lod4->addChild(m_tilesLod4);*/

    //osg::ref_ptr<osg::Geode> geode = buildGrid(osg::Vec3(64300.0, 6861500.0, 0.0), 500.0, 10);
    osg::ref_ptr<osg::Geode> grid = buildGrid(osg::Vec3(0.0, 0.0, 0.0), 500.0, 30);
    //m_layers->addChild(grid);

    //osg::ref_ptr<osg::Geode> bbox = buildBBox(osg::Vec3(100.0, 100.0, 100.0), osg::Vec3(400.0, 400.0, 400.0));
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
int OsgScene::getNbTiles() const
{
    /*osg::ref_ptr<const osg::Group> effect = getChild(0)->asGroup();
    if(effect)
    {
        osg::ref_ptr<const osg::Group> tiles = effect->getChild(0)->asGroup();
        if(tiles)
        {
            return tiles->getNumChildren();
        }
    }*/

    return 0;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Group> OsgScene::getTile(int /*id*/)
{
    /*osg::ref_ptr<osg::Group> effect = getChild(0)->asGroup();
    if(effect)
    {
        osg::ref_ptr<osg::Group> tiles = effect->getChild(0)->asGroup();
        if(tiles)
        {
            return tiles->getChild(id)->asGroup();
        }
    }*/

    return NULL;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Group> OsgScene::getTile(const std::string& /*name*/)
{
    return NULL;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Node> OsgScene::findNode(const std::string& name)
{
    FindNamedNode f(name);
    accept(f);
    return f.getNode();
    //return NULL;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Node> findNode(const vcity::URI& /*uri*/)
{
    return NULL;
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::deleteNode(const std::string& name)
{
    osg::ref_ptr<osg::Node> node = findNode(name);

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
void setYearRec(int year, osg::ref_ptr<osg::Node> node)
{
    osg::ref_ptr<osg::Group> grp = node->asGroup();
    if(grp)
    {
        for(unsigned int i=0; i<grp->getNumChildren(); ++i)
        {
            osg::ref_ptr<osg::Node> child = grp->getChild(i);

            int yearOfConstruction;
            int yearOfDemolition;

            bool a = node->getUserValue("yearOfConstruction", yearOfConstruction);
            bool b = node->getUserValue("yearOfDemolition", yearOfDemolition);

            //std::cout << node->getName() << " : " << a <<  " : yearOfConstruction : " << yearOfConstruction << std::endl;
            //std::cout << node->getName() << " : " << b << " : yearOfDemolition : " << yearOfDemolition << std::endl;

            if(a && b)
            {
                if((yearOfConstruction < year && year < yearOfDemolition) || year == -1)
                {
                    node->setNodeMask(0xffffffff);
                }
                else
                {
                     node->setNodeMask(0);
                }
                //node->setNodeMask(0xffffffff - node->getNodeMask());
            }



            if(year == -1)
            {
                node->setNodeMask(0xffffffff);
            }

            setYearRec(year, child);
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
void OsgScene::setYear(int year)
{
    setYearRec(year, this);
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
void OsgScene::showNode(osg::ref_ptr<osg::Node> node, bool show)
{
    if(node)
    {
        node->setNodeMask(0xffffffff - node->getNodeMask());
        /*if(show)
        {
            node->setNodeMask(~0x0);
        }
        else
        {
            node->setNodeMask(0x0);
        }*/
    }
}
////////////////////////////////////////////////////////////////////////////////
void OsgScene::showNode(const vcity::URI& uri, bool show)
{
    showNode(getNode(uri), show);
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

////////////////////////////////////////////////////////////////////////////////
void OsgScene::buildTileRec(osg::ref_ptr<osg::Group> nodeOsg, citygml::CityObject* node, int depth)
{
    citygml::CityObjects& cityObjects = node->getChildren();
    citygml::CityObjects::iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        nodeOsg->addChild((*it)->getOsgNode());
        buildTileRec((*it)->getOsgNode(), *it, depth+1);
    }
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Node> OsgScene::buildTile(const vcity::Tile& tile)
{
    //osg::ref_ptr<osg::Group> grp = new osg::Group();
    osg::ref_ptr<osg::PositionAttitudeTransform> root = new osg::PositionAttitudeTransform();
    root->setName(tile.getName());
    //const TVec3d& t = m_root->getTranslationParameters();
    //const TVec3d& t = (tile.getCityModel()->getEnvelope().getLowerBound() + tile.getCityModel()->getEnvelope().getUpperBound())/2;
    //const TVec3d& t = tile.getCityModel()->getEnvelope().getUpperBound();
    //const TVec3d& t = tile.getCityModel()->getTranslationParameters();
    //t = t + tile.getCityModel()->getTranslationParameters();
    //osg::Vec3d pos = osg::Vec3d(t.x, t.y, 0);
    //root->setPosition(pos);

    //std::cout << "tile pos : " << t.x << ", " << t.y << ", " << t.z << std::endl;

    const citygml::CityObjects& cityObjects = tile.getCityModel()->getCityObjectsRoots();
    citygml::CityObjects::const_iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        osg::ref_ptr<osg::Group> node = (*it)->getOsgNode();
        root->addChild(node);
        //root->setUserValue("citygml", tile.getCityModel());
        buildTileRec(node, *it);
    }

    //dumpOsgTree(root);

    //m_rootOsg = root;
    return root;
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Node> OsgScene::getNode(const vcity::URI& uri)
{
    FindNamedNode f(uri.getLastNode());
    accept(f);
    return f.getNode();
}
////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Geode> OsgScene::buildGrid(osg::Vec3 origin, float step, int n)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

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
/*osg::ref_ptr<osg::Geode> OsgScene::buildBBox(osg::Vec3 lowerBound, osg::Vec3 upperBound)
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
}*/
////////////////////////////////////////////////////////////////////////////////
