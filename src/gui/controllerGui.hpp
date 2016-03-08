// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __CONTROLLERGUI_HPP__
#define __CONTROLLERGUI_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "core/controller.hpp"
////////////////////////////////////////////////////////////////////////////////
class ControllerGui : public vcity::Controller
{
public:
    ControllerGui();
    virtual ~ControllerGui() override;

    virtual void reset() override;

    virtual void addNode(const vcity::URI& uri) override;
    virtual void deleteNode(const vcity::URI& uri) override;

    // layer
    virtual void addLayer(const std::string& name) override;
    virtual void deleteLayer(const vcity::URI& uri) override;
    virtual void setLayerName(const vcity::URI& uri, const std::string& name) override;

    // tile
    virtual void addTile(const vcity::URI& uriLayer, vcity::Tile& tile) override;
    virtual void deleteTile(const vcity::URI& uri) override;
    virtual void setTileName(const vcity::URI& uri, const std::string& name) override;

    //info
    virtual void addInfo(const vcity::URI& uriLayer, std::vector<osgInfo*> info);

    // temporal
    virtual void addTag(const vcity::URI& uri, citygml::CityObjectTag* tag) override;
    virtual void addState(const vcity::URI& uri, citygml::CityObjectState* state) override;
    virtual void addDynState(const vcity::URI& uri, citygml::CityObjectDynState* state) override;

	// Assimp
	virtual void addAssimpNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node);
	virtual void deleteAssimpNode(const vcity::URI& uri);
	virtual void setAssimpNodeName(const vcity::URI& uri, const std::string& name);

	// MntAsc
	virtual void addMntAscNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node);

	// Las
	virtual void addLasNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node);

    // Shp
    virtual void addShpNode(const vcity::URI& uriLayer, OGRDataSource* poDS) override;

    // selection
    virtual void resetSelection() override;
    virtual bool addSelection(const vcity::URI& uri) override;

    // update a node after model modifs (update tree & osg)
    void update(const vcity::URI& uri);

private:
};
////////////////////////////////////////////////////////////////////////////////
#endif // __CONTROLLERGUI_HPP__
