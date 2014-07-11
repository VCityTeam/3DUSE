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

    virtual void reset();

    virtual void addNode(const vcity::URI& uri);
    virtual void deleteNode(const vcity::URI& uri);

    // layer
    virtual void addLayer(const std::string& name);
    virtual void deleteLayer(const vcity::URI& uri);
    virtual void setLayerName(const vcity::URI& uri, const std::string& name);

    // tile
    virtual void addTile(const vcity::URI& uriLayer, vcity::Tile& tile);
    virtual void deleteTile(const vcity::URI& uri);
    virtual void setTileName(const vcity::URI& uri, const std::string& name);

	// Assimp
	virtual void addAssimpNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node);
	virtual void deleteAssimpNode(const vcity::URI& uri);
	virtual void setAssimpNodeName(const vcity::URI& uri, const std::string& name);

	// MntAsc
	virtual void addMntAscNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node);

    // Shp
    virtual void addShpNode(const vcity::URI& uriLayer, OGRDataSource* poDS);

    // selection
    virtual void resetSelection();
    virtual bool addSelection(const vcity::URI& uri) override;

    // update a node after model modifs (update tree & osg)
    void update(const vcity::URI& uri);

private:
};
////////////////////////////////////////////////////////////////////////////////
#endif // __CONTROLLERGUI_HPP__
