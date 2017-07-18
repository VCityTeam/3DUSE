// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef REFERENCE_HPP
#define REFERENCE_HPP

#include "cityobject.hpp"
#include <ctime>
#include "documentObject.hpp"

namespace documentADE
{
  enum PurposeType {
      PT_EXISTING=0,
      PT_NEW_PROJECT
  };
  class CoveragePeriod {
  public:
      void setStartTime(time_t startTime);
      void setEndTime(time_t startTime);
      time_t getStartTime();
      time_t getEndTime();
  private:
      time_t startTime;
      time_t endTime;
  };
  class Reference: public citygml::Object
  {
  public:
      Reference(const std::string& id);
      time_t getReferringDate();
      std::string getReferenceText();
      std::string getPurpose();
      PurposeType getPurposeType();
      CoveragePeriod getCoveragePeriod();
      DocumentObject* getReferenceDocumentObject();
      citygml::CityObject* getReferencedCityObject();

      void setReferringDate(time_t referringDate);
      void setReferenceText(std::string referenceText);
      void setPurpose(std::string purpose);
      void setPurposeType(PurposeType purpseType);
      void setCoveragePeriod(CoveragePeriod coveragePeriod);
      void setReferenceDocument(DocumentObject*);
      void setReferencedCityObject(citygml::CityObject*);
  private:
      citygml::CityObject *_cityObject;
      DocumentObject *_documentObject;
      time_t _referringDate;
      std::string _referenceText;
      std::string _purpose;
      PurposeType _purposeType;
      CoveragePeriod _coveragePeriod;

  };
}

#endif // REFERENCE_HPP

