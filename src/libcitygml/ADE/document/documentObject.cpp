#include "documentObject.hpp"

namespace documentADE
{

   void DocumentObject::setTitle(std::string title)
   {
       _title = title;
   }
   void DocumentObject::setTheme(std::string theme)
   {
       _theme = theme;
   }
   void DocumentObject::setClass(std::string cclass)
   {
       _class = cclass;
   }
   void DocumentObject::setFunction(std::string function)
   {
       _function = function;
   }
   void DocumentObject::setUsage(std::string usage)
   {
       _usage = usage;
   }
   void DocumentObject::setDescription(std::string description)
   {
       _description = description;
   }
   void DocumentObject::setMandate(std::string mandate)
   {
       _mandate = mandate;
   }
   void DocumentObject::setCreator(std::string creator)
   {
       _creator = creator;
   }
   void DocumentObject::setPublisher(std::string publisher)
   {
       _publisher = publisher;
   }
   void DocumentObject::setCurrentKnownPossessor(std::string currentKnownPossessor)
   {
       _currentKnownPossessor = currentKnownPossessor;
   }
   void DocumentObject::setCurrentRightsHolder(std::string currentRightsHolder)
   {
       _currentRightsHolder = currentRightsHolder;
   }
   void DocumentObject::setCurrentPossessionDate(QDateTime currentPossessionDate)
   {
       _currentPossessionDate = currentPossessionDate;
   }
   void DocumentObject::setCurrentRightsObtainedDate(QDateTime currentRightsObtainedDate)
   {
       _currentRightsObtainedDate = currentRightsObtainedDate;
   }
   void DocumentObject::setPublicationDate(QDateTime publicationDate)
   {
       _publicationDate = publicationDate;
   }
   void DocumentObject::setDocumentType(DocumentType documentType)
   {
       _documentType = documentType;
   }
   void DocumentObject::setRights(RightsType rights)
   {
       _rights = rights;
   }
   void DocumentObject::setFormat(FormatType format)
   {
       _format = format;
   }
   void DocumentObject::setThemeType(ThemeType themeType)
   {
       _themeType = themeType;
   }
   void DocumentObject::setCreatorType(CreatorType creatorType)
   {
       _creatorType = creatorType;
   }
   void DocumentObject::setCurrentKnownPossessorType(CurrentKnownPossessorType
                                                     currentKnownPossessorType)
   {
       _currentKnownPossessorType = currentKnownPossessorType;
   }
   void DocumentObject::setCurrentRightsHolderType(CurrentRightsHolderType
                                                   currentRightsHolderType)
   {
       _currentRightsHolderType = currentRightsHolderType;
   }

   void DocumentObject::addTag(Tag* tag) {
       _tags.push_back(tag);
   }

   std::string DocumentObject::getTitle()
   {
       return _title;
   }
   std::string DocumentObject::getTheme()
   {
       return _theme;
   }
   std::string DocumentObject::getClass()
   {
       return _class;
   }
   std::string DocumentObject::getFunction()
   {
       return _function;
   }
   std::string DocumentObject::getUsage()
   {
       return _usage;
   }
   std::string DocumentObject::getDescription()
   {
       return _description;
   }
   std::string DocumentObject::getMandate()
   {
       return _mandate;
   }
   std::string DocumentObject::getCreator()
   {
       return _creator;
   }
   std::string DocumentObject::getPublisher()
   {
       return _publisher;
   }
   std::string DocumentObject::getCurrentKnownPossessor()
   {
       return _currentKnownPossessor;
   }
   std::string DocumentObject::getCurrentRightsHolder()
   {
       return _currentRightsHolder;
   }
   QDateTime DocumentObject::getCurrentPossessionDate()
   {
       return _currentPossessionDate;
   }
   QDateTime DocumentObject::getCurrentRightsObtainedDate()
   {
       return _currentRightsObtainedDate;
   }
   QDateTime DocumentObject::getPublicationDate()
   {
       return _publicationDate;
   }
   DocumentType DocumentObject::getDocumentType()
   {
       return _documentType;
   }
   RightsType DocumentObject::getRights()
   {
       return _rights;
   }
   FormatType DocumentObject::getFormat()
   {
       return _format;
   }
   ThemeType DocumentObject::getThemeType()
   {
       return _themeType;
   }
   CreatorType DocumentObject::getCreatorType()
   {
       return _creatorType;
   }
   CurrentKnownPossessorType DocumentObject::getCurrentKnownPossessorType()
   {
       return _currentKnownPossessorType;
   }
   CurrentRightsHolderType DocumentObject::getCurrentRightsHolderType()
   {
       return _currentRightsHolderType;
   }

   std::vector<Tag*> DocumentObject::getTags()
   {
       return _tags;
   }
}
