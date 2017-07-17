// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __LayerMnt_HPP__
#define __LayerMnt_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "abstractlayer.hpp"

#include "URI.hpp"
#include <string>
#include <memory>
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
    ////////////////////////////////////////////////////////////////////////////////
    /// \brief LayerMnt class : it holds all the assimp objects
    class LayerMnt : public abstractLayer
    {
    public:
        /// \brief Layer Build empty layer
        /// \param name Layer name
        LayerMnt(const std::string& name);

        virtual ~LayerMnt() override;

        /// Get layer type as string
        const std::string getType() const override;

        /// Get Layer URI
        URI getURI() const override;

        void dump() override;

    private:
    };
    ////////////////////////////////////////////////////////////////////////////////
    typedef std::shared_ptr<LayerMnt> LayerMntPtr;
    ////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __LayerMnt_HPP__
