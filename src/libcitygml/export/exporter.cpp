// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "exporter.hpp"

namespace citygml
{

Exporter::Exporter()
   : m_temporalExport(false), m_date(), m_offsetX(0.0), m_offsetY(0.0)
{
}

Exporter::~Exporter()
{
}

void Exporter::setTemporalExport(bool param)
{
   m_temporalExport = param;
}

void Exporter::setDate(const boost::posix_time::ptime& date)
{
   m_date = date;
}

void Exporter::setOffset(double offsetX, double offsetY)
{
   m_offsetX = offsetX;
   m_offsetY = offsetY;
}

} // namespace citygml
