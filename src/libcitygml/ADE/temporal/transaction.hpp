#ifndef _TRANSACTION_HPP_
#define _TRANSACTION_HPP_

#include "cityobject.hpp"

namespace temporal
{

enum TransactionValue
{
	INSERT,
	DEL,
	REPLACE
};
class Transaction: public citygml::Object
{
public:
	Transaction(const std::string& id);
	void setType(TransactionValue);
	void setOldFeature(citygml::CityObject*);
	void setNewFeature(citygml::CityObject*);
private:
	TransactionValue _type;
	citygml::CityObject* _newFeature;
	citygml::CityObject* _oldFeature;
};

} //namespace temporal

#endif //_TRANSACTION_HPP_
