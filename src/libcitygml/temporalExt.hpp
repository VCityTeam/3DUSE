#ifndef __TEMPORALEXT_H__
#define __TEMPORALEXT_H__
////////////////////////////////////////////////////////////////////////////////
#include <QDateTime>
#include <map>
#include <string>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
class CityObject;

class BuildingTag
{
public:
    BuildingTag(int year, CityObject* geom);

    int getId() const;
    //std::string getStringId() const { std::stringstream ss; ss << "TAG" << m_id << m_name; return ""; /*ss.str();*/ }
    std::string getStringId() const;
    //std::string getGeomName() const { std::string a = m_geom?m_geom->getId():0; return getStringId()+a; }
    CityObject* getGeom();

    const std::string& getAttribute(const std::string& attribName, QDateTime date) const;

    int m_year;
    QDateTime m_date;
    int m_id;
    std::string m_name;
    CityObject* m_parent;

private:

    CityObject* m_geom;
    //int m_year;
};
////////////////////////////////////////////////////////////////////////////////
class BuildingFlag
{
public:
    BuildingFlag(CityObject* geom);

    int getId() const;
    //std::string getStringId() const { std::stringstream ss; ss << "FLAG" << m_id << m_name; return "";/*ss.str();*/ }
    std::string getStringId() const;
    //std::string getGeomName() const { std::string a = m_geom?m_geom->getId():0; return getStringId()+a; }
    CityObject* getGeom();

    const std::string& getAttribute(const std::string& attribName, QDateTime date) const;

    std::map<std::string, std::string> m_attributes;    ///< attributes map

    int m_id;
    std::string m_name;
    CityObject* m_parent;

private:

    CityObject* m_geom;
};
////////////////////////////////////////////////////////////////////////////////
class DataSource
{
public:
    virtual const std::string& getAttribute(const QDateTime& date) const=0;
};
////////////////////////////////////////////////////////////////////////////////
class DataSourceArray : public DataSource
{
public:
    virtual const std::string& getAttribute(const QDateTime& date) const;
};
////////////////////////////////////////////////////////////////////////////////
class DataSourceFile : public DataSource
{
public:
    virtual const std::string& getAttribute(const QDateTime& date) const;
};
////////////////////////////////////////////////////////////////////////////////
class BuildingDynFlag : public BuildingFlag
{
public:
    BuildingDynFlag(CityObject* geom);
    ~BuildingDynFlag();

    void addDataSource(DataSource* dsrc);

    const std::string& getAttribute(const std::string& attribName, const QDateTime& date) const;

private:
    std::map<std::string, DataSource*> m_attributes;    ///< attributes map
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __TEMPORALEXT_H__
