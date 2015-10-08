#ifndef _TEMPORALHANDLER_HPP_
#define _TEMPORALHANDLER_HPP_

#include "../ADE.hpp"
#include "version.hpp"
#include "versionTransition.hpp"
#include "transaction.hpp"

class TempHandler : public ADEHandler
{
protected :
	temporal::Version* _currentVersion;
	temporal::VersionTransition* _currentTransition;
	temporal::Transaction* _currentTransaction;
	bool _inFromTags;
	bool _inToTags;
	std::vector<temporal::Version*> _versions;
	std::vector<temporal::VersionTransition*> _transitions;

public:
	TempHandler(void);
	TempHandler(citygml::CityGMLHandler* gmlHandler);
	void startElement(std::string, void*);
	void endElement(std::string);
	void endDocument();
protected:
	std::string removeNamespace(std::string );
	std::string getIDfromQuery(std::string);
private:
	//Adding to ADE register (template in ADE.hpp)
	static ADERegister<TempHandler> reg;
};

#endif
