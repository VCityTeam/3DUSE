// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __LAYERINFO_HPP__
#define __LAYERINFO_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "abstractlayer.hpp"
#include "URI.hpp"
#include <string>
#include <memory>
#include "ogrsf_frmts.h"
#include "gui/osg/osgInfo.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
/// \brief LayerInfo class : it holds shp objects
class LayerInfo : public abstractLayer
{
public:
    /// \brief Layer Build empty layer
    /// \param name Layer name
    LayerInfo(const std::string& name);

    virtual ~LayerInfo() override;

    /// Get layer type as string
	const std::string getType() const override;

    /// Get Layer URI
	URI getURI() const override;

    void setInfo(std::vector<osgInfo*> info);
    std::vector<osgInfo*> getInfo();

    /// \brief getCityObjectNode Get a CityGML node
    /// \param uri URI pointing to the CityGML node
    /// \return Ptr to CityGML node or nullptr
    //osg::Info* getInfoNode(const URI& uri);

    void dump() override;

public:
    std::vector<osgInfo*> m_info;
};
////////////////////////////////////////////////////////////////////////////////
typedef std::shared_ptr<LayerInfo> LayerShpInfo;
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __LAYERINFO_HPP__

