#ifndef _VERSION_TRANSITION_HPP_
#define _VERSION_TRANSITION_HPP_

#include "transaction.hpp"

namespace temporal
{

enum TransitionValue
{
	planned,
	realized,
	historical_succession,
	fork,
	merge
};

class VersionTransition : public citygml::Object
{
public:
	VersionTransition (const std::string& id);
	virtual ~VersionTransition() override;

private:

	bool _clonePredecessor;
	std::string _reason;
	TransitionValue _type;

	citygml::CityObject* _from;
	citygml::CityObject* _to;

	std::vector<Transaction*> _transactions;
};

} //namespace temporal

#endif //_VERSION_TRANSITION_HPP_