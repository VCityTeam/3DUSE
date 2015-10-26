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
