// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DOCUMENTOBJECT_HPP
#define DOCUMENTOBJECT_HPP

#include "../../cityobject.hpp"
#include <ctime>
#include "tag.hpp"

namespace documentADE
{

#define MAKE_RGBA( _r_, _g_, _b_, _a_ ) TVec4f( _r_/255.f, _g_/255.f, _b_/255.f, _a_/255.f )
#define MAKE_RGB( _r_, _g_, _b_ ) MAKE_RGBA( _r_, _g_, _b_, 255 )

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

  class DocumentObject:  public citygml::CityObject
  {
  public:
      DocumentObject( const std::string& id ) : citygml::CityObject( id, citygml::COT_GenericCityObject ){}
      inline TVec4f getDefaultColor( void ) const
      {
         return MAKE_RGB( 10, 230, 1 );
      }
      void setTitle(std::string);
      void setTheme(std::string);
      void setClass(std::string );
      void setFunction(std::string);
      void setUsage(std::string);
      void setDescription(std::string);
      void setMandate(std::string );
      void setCreator(std::string );
      void setPublisher(std::string);
      void setCurrentKnownPossessor(std::string);
      void setCurrentRightsHolder(std::string);
      void setCurrentPossessionDate(time_t);
      void setCurrentRightsObtainedDate(time_t);
      void setPublicationDate(time_t);
      void setDocumentType(DocumentType);
      void setRights(RightsType);
      void setFormat(FormatType);
      void setThemeType(ThemeType);
      void setCreatorType(CreatorType);
      void setCurrentKnownPossessorType(CurrentKnownPossessorType);
      void setCurrentRightsHolderType(CurrentRightsHolderType);
      std::vector<Tag*> getTags();

      std::string getTitle();
      std::string getTheme();
      std::string getClass();
      std::string getFunction();
      std::string getUsage();
      std::string getDescription();
      std::string getMandate();
      std::string getCreator();
      std::string getPublisher();
      std::string getCurrentKnownPossessor();
      std::string getCurrentRightsHolder();
      time_t getCurrentPossessionDate();
      time_t getCurrentRightsObtainedDate();
      time_t getPublicationDate();
      DocumentType getDocumentType();
      RightsType getRights();
      FormatType getFormat();
      ThemeType getThemeType();
      CreatorType getCreatorType();
      CurrentKnownPossessorType getCurrentKnownPossessorType();
      CurrentRightsHolderType getCurrentRightsHolderType();
      void addTag(Tag*);

  private:
    std::string _title;
    std::string _theme;
    std::string _class;
    std::string _function;
    std::string _usage;
    std::string _description;
    std::string _mandate;
    time_t _publicationDate;
    std::string _creator;
    std::string _publisher;
    std::string _currentKnownPossessor;
    std::string _currentRightsHolder;
    time_t _currentPossessionDate;
    time_t _currentRightsObtainedDate;
    DocumentType _documentType;
    RightsType _rights;
    FormatType _format;
    ThemeType _themeType;
    CreatorType _creatorType;
    CurrentKnownPossessorType _currentKnownPossessorType;
    CurrentRightsHolderType _currentRightsHolderType;
    std::vector<Tag*> _tags;
    vcity::URI _linkToCreator;
  };
}
#endif // DOCUMENTOBJECT_HPP

