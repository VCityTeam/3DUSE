#ifndef _TRANSACTION_HPP_
#define _TRANSACTION_HPP_

#include "citygml.hpp"

namespace temporal
{

enum TransactionValue
{
	insert,
	del,
	replace
};

class Transaction
{
public:
	Transaction();
private:
	TransactionValue _type;
	citygml::CityObject* _newFeature;
	citygml::CityObject* _oldFeature;
};

} //namespace temporal

#endif //_TRANSACTION_HPP_