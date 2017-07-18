// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __CITYGML_IMPORTER_HPP__
#define __CITYGML_IMPORTER_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "citygml_export.h"

namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief Base class for data import
///
class CITYGML_EXPORT Importer
{
public:
    Importer();
    virtual ~Importer();

    /// Set offset, to be added to loaded data (to counteract dataprofile offset)
    void setOffset(double offsetX, double offsetY);

protected:

    double m_offsetX;   ///< x offset
    double m_offsetY;   ///< y offset

private:
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_IMPORTER_HPP__
