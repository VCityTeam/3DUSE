#ifndef __TEMPORALEXT_HPP__
#define __TEMPORALEXT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <QDateTime>
#include <map>
#include <string>
#include <osg/Group>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class CityObject;
class CityObjectState;
////////////////////////////////////////////////////////////////////////////////
enum State {
        Build = 1 << 0,
        Destroyed = 1 << 1,
        Modified = 1 << 2,
        Burn = 1 << 3
};
////////////////////////////////////////////////////////////////////////////////
class CityObjectTag
{
public:
    CityObjectTag(int year, CityObject* geom);

    int getId() const;
    //std::string getStringId() const { std::stringstream ss; ss << "TAG" << m_id << m_name; return ""; /*ss.str();*/ }
    std::string getStringId() const;
    //std::string getGeomName() const { std::string a = m_geom?m_geom->getId():0; return getStringId()+a; }
    CityObject* getGeom();
    const CityObject* getGeom() const;
    const CityObject* getParent() const;
    osg::ref_ptr<osg::Group> getOsg();
    void setOsg(osg::ref_ptr<osg::Group> node);

    std::string getAttribute(const std::string& attribName, const QDateTime& date) const;

    int m_year;
    QDateTime m_date;
    int m_id;
    std::string m_name;
    CityObject* m_parent;

    CityObjectState* m_state;

private:

    CityObject* m_geom;
    osg::ref_ptr<osg::Group> m_osg;
    //int m_year;
};
bool cmpTag(CityObjectTag* a, CityObjectTag* b);
////////////////////////////////////////////////////////////////////////////////
class CityObjectState
{
public:
    CityObjectState(CityObject* geom);
    virtual ~CityObjectState() {}

    int getId() const;
    //std::string getStringId() const { std::stringstream ss; ss << "STATE" << m_id << m_name; return "";/*ss.str();*/ }
    virtual std::string getStringId() const;
    //std::string getGeomName() const { std::string a = m_geom?m_geom->getId():0; return getStringId()+a; }
    CityObject* getGeom();
    const CityObject* getGeom() const;
    const CityObject* getParent() const;

    virtual std::string getAttribute(const std::string& attribName, const QDateTime& date) const;

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
    DataSource(const std::string& attributeName);
    virtual ~DataSource() {}

    virtual std::string getAttribute(const QDateTime& date) const;


   void dump() const;

//protected:
    std::string m_attribute;

    std::vector<QDateTime> m_dates;
    std::vector<std::string> m_values;
};
////////////////////////////////////////////////////////////////////////////////
class DataSourceArray : public DataSource
{
public:
    DataSourceArray(const std::string& attributeName, const std::string& data);
    //virtual std::string getAttribute(const QDateTime& date) const override;
    void parse(const std::string& data);
};
////////////////////////////////////////////////////////////////////////////////
class DataSourceFile : public DataSource
{
public:
    DataSourceFile(const std::string& attributeName, const std::string& filePath);
    //virtual std::string getAttribute(const QDateTime& date) const override;
    void parse(const std::string& filePath);

    std::string m_filePath;
};
////////////////////////////////////////////////////////////////////////////////
class CityObjectDynState : public CityObjectState
{
public:
    CityObjectDynState(CityObject* geom);
    virtual ~CityObjectDynState() override {}

    virtual std::string getStringId() const;

    void addDataSource(DataSource* dsrc);

    virtual std::string getAttribute(const std::string& attribName, const QDateTime& date) const override;

private:
    std::map<std::string, DataSource*> m_attributes;    ///< attributes map
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __TEMPORALEXT_HPP__
