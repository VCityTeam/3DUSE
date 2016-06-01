/* -*-c++-*- libcitygml - VCity project, 3DUSE, Liris
 * Temporal addons */
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <sstream>
#include "temporalExt.hpp"
#include "cityobject.hpp"

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
////////////////////////////////////////////////////////////////////////////////
const CityObject* CityObjectTag::getGeom() const
{
    return m_geom;
}
////////////////////////////////////////////////////////////////////////////////
const CityObject* CityObjectTag::getParent() const
{
    return m_parent;
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
std::string CityObjectTag::getAttribute(const std::string& attribName, const QDateTime& date) const
{
    return m_state->getAttribute(attribName, date);
}
////////////////////////////////////////////////////////////////////////////////
CityObjectState::CityObjectState(CityObject* geom)
    : m_geom(geom)
{

}
////////////////////////////////////////////////////////////////////////////////
CityObjectState::~CityObjectState()
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
const CityObject* CityObjectState::getGeom() const
{
    return m_geom;
}
////////////////////////////////////////////////////////////////////////////////
const CityObject* CityObjectState::getParent() const
{
    return m_parent;
}
////////////////////////////////////////////////////////////////////////////////
std::string CityObjectState::getAttribute(const std::string& attribName, const QDateTime& /*date*/) const
{
    // first look in state attribs
    const auto& it = m_attributes.find(attribName);
    if(it==m_attributes.end())
    {
        // next look in cityobject attribs
        return m_geom->getAttribute(attribName);
    }
    else
    {
        return it->second;
    }
    //std::cout << "BuildingFlag::getAttribute" << std::endl;
    return "";
}
////////////////////////////////////////////////////////////////////////////////
DataSource::DataSource(const std::string& attributeName)
    : m_attribute(attributeName)
{

}
////////////////////////////////////////////////////////////////////////////////
DataSource::~DataSource()
{
}
////////////////////////////////////////////////////////////////////////////////
std::string DataSource::getAttribute(const QDateTime& date) const
{
    //std::cout << "DataSource::getAttribute" << std::endl;
    if(m_dates.size() == 1)
    {
        if(date > m_dates[0])
            return m_values[0];
    }

    // find date interval
    for(size_t i=0; i<m_dates.size()-1; ++i)
    {
        if(m_dates[i] < date && date < m_dates[i+1])
            return m_values[i];
    }
    return "";
}
////////////////////////////////////////////////////////////////////////////////
void DataSource::dump() const
{
    for(size_t i=0; i<m_dates.size(); ++i)
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
// helper function to read dates allowing multiple formats
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

    // read data :
    // format : date(format : yyyy/MM/dd-HH:mm:ss) | value (separator : |)
    std::stringstream ss(data);
    std::string item;
    bool date = true;
    while(std::getline(ss, item, '|')) // split data using separator |
    {
        if(date)
        {
            m_dates.push_back(getDate(item));
        }
        else
        {
            m_values.push_back(item);
        }
        date = !date; // alternatively read a date and a value
    }
}
////////////////////////////////////////////////////////////////////////////////
/*std::string DataSourceArray::getAttribute(const QDateTime& date) const
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

    // read data :
    // format :
    // date1(format : yyyy/MM/dd-HH:mm:ss)
    // value1
    // date2(format : yyyy/MM/dd-HH:mm:ss)
    // value2
    // date3(format : yyyy/MM/dd-HH:mm:ss)
    // value3
    ///...
    std::string item;
    std::ifstream file(filePath);
    bool date = true;
    while(file >> item) // read line by line
    {
        if(date)
        {
            m_dates.push_back(getDate(item));
        }
        else
        {
            m_values.push_back(item);
        }
        date = !date; // alternatively read a date and a value
    }
}
////////////////////////////////////////////////////////////////////////////////
/*std::string DataSourceFile::getAttribute(const QDateTime& date) const
{
}*/
////////////////////////////////////////////////////////////////////////////////
CityObjectDynState::CityObjectDynState(CityObject* geom)
    : CityObjectState(geom)
{
}
////////////////////////////////////////////////////////////////////////////////
CityObjectDynState::~CityObjectDynState()
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
std::string CityObjectDynState::getAttribute(const std::string& attribName, const QDateTime& date) const
{
    //std::cout << "BuildingDynFlag::getAttribute" << std::endl;
    std::map<std::string, DataSource*>::const_iterator it = m_attributes.find(attribName);
    if(it != m_attributes.end())
    {
        return it->second->getAttribute(date);
    }

    return "";
}
////////////////////////////////////////////////////////////////////////////////
bool cmpTag(CityObjectTag* a, CityObjectTag* b)
{
    return (a->m_date < b->m_date);
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
