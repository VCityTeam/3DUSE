// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "transaction.hpp"

namespace temporal
{

Transaction::Transaction( const std::string& id ) : Object( id )
{}

void Transaction::setType(TransactionValue param)
{
	_type = param;
}

void Transaction::setOldFeature(citygml::CityObject* object)
{
	_oldFeature = object;
}

void Transaction::setNewFeature(citygml::CityObject* object)
{
	_newFeature = object;
}


}//namespace temporal
