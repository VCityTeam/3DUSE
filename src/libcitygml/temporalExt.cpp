#include "temporalExt.hpp"
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
BuildingTag::BuildingTag(int year, CityObject* geom)
    : m_year(year), m_geom(geom)
{

}
////////////////////////////////////////////////////////////////////////////////
int BuildingTag::getId() const
{
    return m_id;
}
////////////////////////////////////////////////////////////////////////////////
std::string BuildingTag::getStringId() const
{
    std::stringstream ss; ss << "TAG" << m_id << m_name;
    return ss.str();
}
////////////////////////////////////////////////////////////////////////////////
CityObject* BuildingTag::getGeom()
{
    return m_geom;
}
/////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Group> BuildingTag::getOsg()
{
    return m_osg;
}
////////////////////////////////////////////////////////////////////////////////
void BuildingTag::setOsg(osg::ref_ptr<osg::Group> node)
{
    m_osg = node;
}
////////////////////////////////////////////////////////////////////////////////
BuildingFlag::BuildingFlag(CityObject* geom)
    : m_geom(geom)
{

}
////////////////////////////////////////////////////////////////////////////////
int BuildingFlag::getId() const
{
    return m_id;
}
////////////////////////////////////////////////////////////////////////////////
std::string BuildingFlag::getStringId() const
{
    std::stringstream ss; ss << "FLAG" << m_id << m_name;
    return ss.str();
}
////////////////////////////////////////////////////////////////////////////////
CityObject* BuildingFlag::getGeom()
{
    return m_geom;
}
////////////////////////////////////////////////////////////////////////////////
BuildingDynFlag::BuildingDynFlag(CityObject* geom)
    : BuildingFlag(geom)
{

}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
