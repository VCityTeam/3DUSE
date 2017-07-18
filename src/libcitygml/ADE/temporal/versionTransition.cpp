// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
