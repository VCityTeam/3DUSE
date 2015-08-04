#include "version.hpp"

namespace temporal
{

Version::Version( const std::string& id ) : Object( id )
{}

const std::vector<std::string>& Version::getTags()
{
	return _tags;
}

const std::vector<citygml::CityObject*>& Version::getVersionMembers()
{
	return _versionMembers;
}


} //namespace temporal