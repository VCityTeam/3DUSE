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
/// \brief LayerShp class : it holds shp objects
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

