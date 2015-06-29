#include "ADE.hpp"

ADEHandler::ADEHandler(void)
{
}

ADEHandler::~ADEHandler(void)
{
}

ADEHandler::ADEHandler(citygml::CityGMLHandler* gHandler)
{
	gmlHandler = gHandler;
}