#ifndef _VERSION_HPP_
#define _VERSION_HPP_

#include "cityobject.hpp"
#include "citygml_export.h"
#pragma warning(disable: 4251) // VC++ DLL jejune complains on STL members

namespace temporal
{
class TempHandler;

class CITYGML_EXPORT Version : public citygml::Object
{
	friend class TempHandler;
public :

	Version(const std::string& id);

	const std::vector<std::string>& getTags();
	void addTag( std::string );

	std::vector<citygml::CityObject*>* getVersionMembers();

	void addMember(citygml::CityObject*);

protected :
	std::vector<std::string> _tags;
	std::vector<citygml::CityObject*> _versionMembers;

};

} //namespace temporal


#endif //_VERSION_HPP_
