#include "version.hpp"

namespace temporal
{

Version::Version( const std::string& id ) : Object( id )
{}

const std::vector<std::string>& Version::getTags()
{
	return _tags;
}

void Version::addTag( std::string tag)
{
	_tags.push_back(tag);
}

std::vector<citygml::CityObject*>* Version::getVersionMembers()
{
	return &_versionMembers;
}

void Version::addMember(citygml::CityObject* object)
{
	_versionMembers.push_back(object);
}

} //namespace temporal
