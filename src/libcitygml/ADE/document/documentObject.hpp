#ifndef DOCUMENTOBJECT_HPP
#define DOCUMENTOBJECT_HPP

namespace documentADE
{

  enum DocumentType
  {
      DT_POSTAL_CARD=0,
      DT_MUNICIPAL_COUNCIL_REPORT
  };
  enum FormatType
  {
      FT_PNG=0,
      FT_JPEG
  };
  enum ThemeType
  {
      TT_PROJECT=0,
      TT_EXISTING
  };
  enum CreatorType
  {
      CT_PUBLIC=0,
      CT_PRIVATE
  };
  enum CurrentRightsHolderType
  {
      CRHT_PUBLIC=0,
      CRHT_PRIVATE
  };
  enum CurrentKnownPossessorType
  {
      CKPT_PUBLIC=0,
      CKPT_PRIVATE
  };
  enum RightsType
  {
      RT_PUBLIC=0,
      RT_PRIVATE
  };
  class DocumentObject:  public citygml::Object
  {
  private:
    std::string _title;
    std::string _theme;
    std::string _class;
    std::string _function;
    std::string _usage;
    std::string _description;
    std::string _mandate;
    QDateTime _publicationDate;
    std::string _creator;
    std::string _publisher;
    std::string _currentKnownPossessor;
    std::string _currentRightsHolder;
    QDateTime _currentPossessionDate;
    QDateTime _currentRightsObtainedDate;
    QDateTime _publicationDate;
    DocumentType _documentType;
    RightsType _rights;
    FormatType _format;
    ThemeType _themeType;
    CreatorType _creatorType;
    CurrentKnownPossessorType _currentKnownPossessorType;
    CurrentRightsHolderType _currentRightsHolderType;
  };
}
#endif // DOCUMENTOBJECT_HPP

