// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __LAYERLAS_HPP__
#define __LAYERLAS_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "abstractlayer.hpp"

#include "URI.hpp"
#include <string>
#include <memory>

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#pragma warning(disable : 4267)
#endif

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
        const std::string getType() const override;

        /// Get Layer URI
        URI getURI() const override;

        void dump() override;

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
