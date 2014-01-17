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

    int m_id;
    std::string m_name;
    CityObject* m_parent;

private:

    CityObject* m_geom;
};
////////////////////////////////////////////////////////////////////////////////
class BuildingDynFlag : public BuildingFlag
{
public:
    BuildingDynFlag(CityObject* geom);

    std::map<QDateTime, std::string> values;
};
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __TEMPORALEXT_H__
