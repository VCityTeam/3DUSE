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

void VersionTransition::setReason(std::string att)
{
	_reason = att;
}

void VersionTransition::setClone(bool att)
{
	_clonePredecessor = att;
}

void VersionTransition::setType(TransitionValue att)
{
	_type = att;
}

void VersionTransition::setFrom(Version* version)
{
	_from = version;
}

Version* VersionTransition::from()
{
	return _from;
}

void VersionTransition::setTo(Version* version)
{
	_to = version;
}

Version* VersionTransition::to()
{
	return _to;
}

void VersionTransition::addTransaction(Transaction* transaction)
{
	_transactions.push_back(transaction);
}

} //namespace temporal
