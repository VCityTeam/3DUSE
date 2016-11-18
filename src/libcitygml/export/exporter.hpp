#ifndef __CITYGML_EXPORTER_HPP__
#define __CITYGML_EXPORTER_HPP__

#include "citygml_export.h"

#ifdef _MSC_VER
#define BOOST_ALL_DYN_LINK 1
#pragma warning(disable: 4251) // To avoid VC++ complaints on missing dll
                               // interface for boost type member m_date
#endif
#include <boost/date_time.hpp>

namespace citygml
{

/// \brief Base class for export
///
class CITYGML_EXPORT Exporter
{
public:
   Exporter();
   virtual ~Exporter();

   /// Enable or disable temporal export
   void setTemporalExport(bool param);

   /// Set temporal export date
   void setDate(const boost::posix_time::ptime& date);

   /// Set offset
   void setOffset(double offsetX, double offsetY);

protected:
   ///< enable temporal export
   bool m_temporalExport;
   ///< date for temporal export
   boost::posix_time::ptime m_date;

   double m_offsetX;       ///< x offset
   double m_offsetY;       ///< y offset

private:
};

} // namespace citygml

#endif // __CITYGML_EXPORTER_HPP__
