#include "temporalExt.hpp"
#include <iostream>
//#include <fstream> // MT 19/03/2014
#include <osgDB/fstream>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
std::string none("none");
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
CityObjectTag::CityObjectTag(int year, CityObject* geom)
    : m_year(year), m_geom(geom)
{

}
////////////////////////////////////////////////////////////////////////////////
int CityObjectTag::getId() const
{
    return m_id;
}
////////////////////////////////////////////////////////////////////////////////
std::string CityObjectTag::getStringId() const
{
    std::stringstream ss; ss << "TAG" << m_id << m_name;
    return ss.str();
}
////////////////////////////////////////////////////////////////////////////////
CityObject* CityObjectTag::getGeom()
{
    return m_geom;
}
/////////////////////////////////////////////////////////////////////////////////
osg::ref_ptr<osg::Group> CityObjectTag::getOsg()
{
    return m_osg;
}
////////////////////////////////////////////////////////////////////////////////
void CityObjectTag::setOsg(osg::ref_ptr<osg::Group> node)
{
    m_osg = node;
}
////////////////////////////////////////////////////////////////////////////////
const std::string& CityObjectTag::getAttribute(const std::string& attribName, const QDateTime& date) const
{
    return m_state->getAttribute(attribName, date);
}
////////////////////////////////////////////////////////////////////////////////
CityObjectState::CityObjectState(CityObject* geom)
    : m_geom(geom)
{

}
////////////////////////////////////////////////////////////////////////////////
int CityObjectState::getId() const
{
    return m_id;
}
////////////////////////////////////////////////////////////////////////////////
std::string CityObjectState::getStringId() const
{
    std::stringstream ss; ss << "STATE" << m_id << m_name;
    return ss.str();
}
////////////////////////////////////////////////////////////////////////////////
CityObject* CityObjectState::getGeom()
{
    return m_geom;
}
////////////////////////////////////////////////////////////////////////////////
const std::string& CityObjectState::getAttribute(const std::string& attribName, const QDateTime& date) const
{
    //std::cout << "BuildingFlag::getAttribute" << std::endl;
    return none;
}
////////////////////////////////////////////////////////////////////////////////
DataSource::DataSource(const std::string& attributeName)
    : m_attribute(attributeName)
{

}
////////////////////////////////////////////////////////////////////////////////
const std::string& DataSource::getAttribute(const QDateTime& date) const
{
    //std::cout << "DataSource::getAttribute" << std::endl;
    if(m_dates.size() == 1)
    {
        if(date > m_dates[0])
            return m_values[0];
    }

    for(int i=0; i<m_dates.size()-1; ++i)
    {
        if(m_dates[i] < date && date < m_dates[i+1])
            return m_values[i];
    }
    return none;
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
QDateTime getDate(const std::string& str)
{
    const char* formats[] = {"yyyy/MM/dd-HH:mm:ss","yyyy/MM/dd-HH:mm","yyyy/MM/dd-HH","yyyy/MM/dd","yyyy/MM","yyyy"};
    QDateTime res;
    for(int i=0; i<6; ++i)
    {
        res = QDateTime::fromString(str.c_str(), formats[i]);
        if(res.isValid())
            break;
    }

    return res;
}
////////////////////////////////////////////////////////////////////////////////
void DataSourceArray::parse(const std::string& data)
{
    std::cout << "add data source array : " << m_attribute << std::endl;
    std::cout << data << std::endl;

    std::stringstream ss(data);
    std::string item;
    bool date = true;
    while(std::getline(ss, item, '|'))
    {
        if(date)
        {
            m_dates.push_back(getDate(item));
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

    std::string item;
    std::ifstream file(filePath);
    bool date = true;
    while(file >> item)
    {
        if(date)
        {
            m_dates.push_back(getDate(item));
        }
        else
        {
            m_values.push_back(item);
        }
        date = !date;
    }
}
////////////////////////////////////////////////////////////////////////////////
/*const std::string& DataSourceFile::getAttribute(const QDateTime& date) const
{
}*/
////////////////////////////////////////////////////////////////////////////////
CityObjectDynState::CityObjectDynState(CityObject* geom)
    : CityObjectState(geom)
{
}
////////////////////////////////////////////////////////////////////////////////
std::string CityObjectDynState::getStringId() const
{
    std::stringstream ss; ss << "DYNSTATE" << m_id << m_name;
    return ss.str();
}
////////////////////////////////////////////////////////////////////////////////
void CityObjectDynState::addDataSource(DataSource* dsrc)
{
    m_attributes[dsrc->m_attribute] = dsrc;
}
////////////////////////////////////////////////////////////////////////////////
const std::string& CityObjectDynState::getAttribute(const std::string& attribName, const QDateTime& date) const
{
    //std::cout << "BuildingDynFlag::getAttribute" << std::endl;
    std::map<std::string, DataSource*>::const_iterator it = m_attributes.find(attribName);
    if(it != m_attributes.end())
    {
        return it->second->getAttribute(date);
    }
}
////////////////////////////////////////////////////////////////////////////////
bool cmpTag(CityObjectTag* a, CityObjectTag* b)
{
    return (a->m_date < b->m_date);
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
