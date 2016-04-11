#include "reference.hpp"

namespace documentADE
{

      QDateTime Reference::getReferringDate()
      {
          return _referringDate;
      }
      std::string Reference::getReferenceText()
      {
          return _referenceText;
      }
      std::string Reference::getPurpose()
      {
          return _purpose;
      }
      PurposeType Reference::getPurpseType()
      {
          return _purpseType;
      }
      CoveragePeriod Reference::getCoveragePeriod()
      {
          return _coveragePeriod;
      }

      void Reference::setReferringDate(QDateTime referringDate)
      {
          this->_referringDate=referringDate;
      }
      void Reference::setReferenceText(std::string referenceText)
      {
          this->_referenceText=referenceText;
      }
      void Reference::setPurpose(std::string purpose)
      {
          this->_purpose=purpose;
      }
      void Reference::setPurpseType(PurposeType purpseType)
      {
          this->_purposeType=purpseType;
      }
      void Reference::setCoveragePeriod(CoveragePeriod coveragePeriod)
      {
          this->_coveragePeriod = coveragePeriod;
      }

}
