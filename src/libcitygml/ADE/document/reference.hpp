#ifndef REFERENCE_HPP
#define REFERENCE_HPP

#include "cityobject.hpp"

namespace documentADE
{
  enum PurposeType {
      PT_EXISTING=0,
      PT_NEW_PROJECT
  };
  class CoveragePeriod {
  public:
      void setStartTime(QDateTime startTime);
      void setEndTime(QDateTime startTime);
      QDateTime getStartTime();
      QDateTime getEndTime();
  private:
      QDateTime startTime;
      QDateTime endTime;
  };
  class Reference: public citygml::Object
  {
  public:
      QDateTime getReferringDate();
      std::string getReferenceText();
      std::string getPurpose();
      PurposeType getPurpseType();
      CoveragePeriod getCoveragePeriod();

      void setReferringDate(QDateTime referringDate);
      void setReferenceText(std::string referenceText);
      void setPurpose(std::string purpose);
      void setPurpseType(PurposeType purpseType);
      void setCoveragePeriod(CoveragePeriod coveragePeriod);
  private:
      QDateTime _referringDate;
      std::string _referenceText;
      std::string _purpose;
      PurposeType _purpseType;
      CoveragePeriod _coveragePeriod;

  };
}

#endif // REFERENCE_HPP

