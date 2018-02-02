// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __LAYERSHP_HPP__
#define __LAYERSHP_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "abstractlayer.hpp"
#include "URI.hpp"
#include <string>
#include <memory>

#ifdef _MSC_VER // remove annoying warnings in Microsoft Visual C++
#	pragma warning(disable:4251)
#endif
#include "ogrsf_frmts.h"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
    ////////////////////////////////////////////////////////////////////////////////
    /// \brief LayerShp class : it holds shp objects
    class LayerShp : public abstractLayer
    {
    public:
        /// \brief Layer Build empty layer
        /// \param name Layer name
        LayerShp(const std::string& name);

        virtual ~LayerShp() override;

        /// Get layer type as string
        const std::string getType() const override;

        /// Get Layer URI
        URI getURI() const override;

        void dump() override;

    public:
        GDALDataset* m_shp;
    };
    ////////////////////////////////////////////////////////////////////////////////
    typedef std::shared_ptr<LayerShp> LayerShpPtr;
    ////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __LAYERSHP_HPP__
