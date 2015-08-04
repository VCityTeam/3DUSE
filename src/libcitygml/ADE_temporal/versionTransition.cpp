#include "versionTransition.hpp"

namespace temporal {

VersionTransition::VersionTransition( const std::string& id ) : Object( id )
{}

VersionTransition::~VersionTransition()
{
	for (Transaction* transaction : _transactions)
	{
		delete transaction;
	}
}

} //namespace temporal