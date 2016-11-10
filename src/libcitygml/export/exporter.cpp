#ifdef _MSC_VER
#pragma warning(disable: 4251) // Concerns complaints on dll interface of
                               //  boost member m_date of Exporter class.
#endif
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
