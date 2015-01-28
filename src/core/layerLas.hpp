// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __LAYERLAS_HPP__
#define __LAYERLAS_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "abstractlayer.hpp"

#include "URI.hpp"
#include <string>
#include <memory>

#ifdef _MSC_VER
#pragma warning(disable : 4996) // TEMP MT
#pragma warning(disable : 4267) // TEMP MT
#endif

// MT : the first time, you must do from /externals/laslib folder :
// mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release; cmake .. -DCMAKE_BUILD_TYPE=Release; make; sudo make install
#include <lasdefinitions.hpp>

////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
struct LayerLas_LASpoint
{
  I32 X;
  I32 Y;
  I32 Z;

  U8 classification : 5;
};

////////////////////////////////////////////////////////////////////////////////
/// \brief LayerLas class : it holds LAS points cloud
class LayerLas : public abstractLayer
{
public:
    /// \brief Layer Build empty layer
    /// \param name Layer name
    LayerLas(const std::string& name);

    virtual ~LayerLas() override;

    /// \brief addLASpoint Add a laspoint in a layer
    /// \param laspoint The laspoint to add
    void addLASpoint(const LASpoint& laspoint);

    /// Get layer type as string
	const std::string getType() const;

    /// Get Layer URI
	URI getURI() const;

    void dump();

	void exportJSON();

private:
    std::vector<LayerLas_LASpoint> m_LASpoints;
};
////////////////////////////////////////////////////////////////////////////////
typedef std::shared_ptr<LayerLas> LayerLasPtr;
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __LAYERLAS_HPP__
