#include "temporalExt.hpp"
#include <iostream>
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
const std::string& BuildingTag::getAttribute(const std::string& attribName, QDateTime date) const
{

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
const std::string& BuildingFlag::getAttribute(const std::string& attribName, QDateTime date) const
{

}
////////////////////////////////////////////////////////////////////////////////
DataSource::DataSource(const std::string& attributeName)
    : m_attribute(attributeName)
{

}
////////////////////////////////////////////////////////////////////////////////
const std::string& DataSource::getAttribute(const QDateTime& date) const
{
    for(int i=0; i<m_dates.size(); ++i)
    {
        if(date > m_dates[i])
            return m_values[i];
    }
}
////////////////////////////////////////////////////////////////////////////////
void DataSource::dump() const
{
    for(int i=0; i<m_dates.size(); ++i)
    {
        std::cout << m_dates[i].toString().toStdString() << " : " << m_values[i] << std::endl;
    }
}
////////////////////////////////////////////////////////////////////////////////
DataSourceArray::DataSourceArray(const std::string& attributeName, const std::string& data)
    : DataSource(attributeName)
{
    parse(data);
    dump();
}
////////////////////////////////////////////////////////////////////////////////
void DataSourceArray::parse(const std::string& data)
{
    std::cout << "add data source array : " << m_attribute << std::endl;
    std::cout << data << std::endl;

    std::stringstream ss(data);
    std::string item;
    bool date = true;
    while (std::getline(ss, item, '|'))
    {
        if(date)
        {
            m_dates.push_back(QDateTime::fromString(item.c_str()));
        }
        else
        {
            m_values.push_back(item);
        }
        date = !date;
    }
}
////////////////////////////////////////////////////////////////////////////////
/*const std::string& DataSourceArray::getAttribute(const QDateTime& date) const
{
}*/
////////////////////////////////////////////////////////////////////////////////
DataSourceFile::DataSourceFile(const std::string& attributeName, const std::string& filePath)
    : DataSource(attributeName), m_filePath(filePath)
{
    parse(filePath);
    dump();
}
////////////////////////////////////////////////////////////////////////////////
void DataSourceFile::parse(const std::string& filePath)
{
    std::cout << "add data source file : " << m_attribute << std::endl;
    std::cout << filePath << std::endl;
}
////////////////////////////////////////////////////////////////////////////////
/*const std::string& DataSourceFile::getAttribute(const QDateTime& date) const
{
}*/
////////////////////////////////////////////////////////////////////////////////
BuildingDynFlag::BuildingDynFlag(CityObject* geom)
    : BuildingFlag(geom)
{

}
////////////////////////////////////////////////////////////////////////////////
void BuildingDynFlag::addDataSource(DataSource* dsrc)
{
    m_attributes[dsrc->m_attribute] = dsrc;
}
////////////////////////////////////////////////////////////////////////////////
const std::string& BuildingDynFlag::getAttribute(const std::string& attribName, const QDateTime& date) const
{
    std::map<std::string, DataSource*>::const_iterator it = m_attributes.find(attribName);
    if(it != m_attributes.end())
    {
        return it->second->getAttribute(date);
    }
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
