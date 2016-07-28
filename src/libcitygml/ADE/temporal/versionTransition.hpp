#ifndef _VERSION_TRANSITION_HPP_
#define _VERSION_TRANSITION_HPP_

#include "transaction.hpp"
#include "version.hpp"
#include "citygml_export.h"

namespace temporal
{
enum TransitionValue
{
	PLANNED,
	REALIZED,
	HISTORICAL_SUCCESSION,
	FORK,
	MERGE
};

class CITYGML_EXPORT VersionTransition : public citygml::Object
{
public:
	VersionTransition (const std::string& id);
	virtual ~VersionTransition() override;

	void setReason(std::string);
	void setClone(bool);
	void setType(TransitionValue);
	void setFrom(Version*);
	Version* from();
	void setTo(Version*);
	Version* to();

	void addTransaction(Transaction*);
	std::vector<Transaction*>* getTransactions();

protected:

	bool _clonePredecessor;
	std::string _reason;
	TransitionValue _type;

	Version* _from;
	Version* _to;

	std::vector<Transaction*> _transactions;
};

} //namespace temporal

#endif //_VERSION_TRANSITION_HPP_
