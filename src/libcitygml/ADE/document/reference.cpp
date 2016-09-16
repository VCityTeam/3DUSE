#include "reference.hpp"

namespace documentADE
{

      Reference::Reference( const std::string& id ) : Object( id )
      {
      }
      time_t Reference::getReferringDate()
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
      PurposeType Reference::getPurposeType()
      {
          return _purposeType;
      }
      CoveragePeriod Reference::getCoveragePeriod()
      {
          return _coveragePeriod;
      }

      void Reference::setReferringDate(time_t referringDate)
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
      void Reference::setPurposeType(PurposeType purposeType)
      {
          this->_purposeType=purposeType;
      }
      void Reference::setCoveragePeriod(CoveragePeriod coveragePeriod)
      {
          this->_coveragePeriod = coveragePeriod;
      }

      void Reference::setReferenceDocument(DocumentObject* documentObject)
      {
          this->_documentObject = documentObject;
      }
      void Reference::setReferencedCityObject(citygml::CityObject* cityObject)
      {
          this->_cityObject = cityObject;
      }
}
