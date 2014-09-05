/* -*-c++-*- libcitygml - VCity project, 3DUSE, Liris
 * Temporal addons */
////////////////////////////////////////////////////////////////////////////////
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
/// \brief The CityObjectTag class
///
///
class CityObjectTag
{
public:
    /// Build a Tag and assign it a geometry
    CityObjectTag(int year, CityObject* geom);

    /// Get Tag id
    int getId() const;
    //std::string getStringId() const { std::stringstream ss; ss << "TAG" << m_id << m_name; return ""; /*ss.str();*/ }

    /// Get Tag name
    std::string getStringId() const;
    //std::string getGeomName() const { std::string a = m_geom?m_geom->getId():0; return getStringId()+a; }

    /// Get Tag geometry
    CityObject* getGeom();

    /// Get Tag geometry (const)
    const CityObject* getGeom() const;

    /// Get parent CityObject (the temporal object)
    const CityObject* getParent() const;

    /// Get osg node
    osg::ref_ptr<osg::Group> getOsg();

    /// Set osg Node
    void setOsg(osg::ref_ptr<osg::Group> node);

    /// Get an attribute for a specific date
    /// \param attribName Attribute name
    /// \param date Date wanted
    std::string getAttribute(const std::string& attribName, const QDateTime& date) const;

    int m_year;                         ///< Year of Tag
    QDateTime m_date;                   ///< Precise date of Tag
    int m_id;                           ///< Tag id (generated)
    std::string m_name;                 ///< Tag name (generated if empty)
    CityObject* m_parent;               ///< Parent object

    CityObjectState* m_state;           ///< Related State

private:
    CityObject* m_geom;                 ///< CityObject describing Tag geometry
    osg::ref_ptr<osg::Group> m_osg;     ///< OSG geometry related to m_geom
    //int m_year;
};
bool cmpTag(CityObjectTag* a, CityObjectTag* b);
////////////////////////////////////////////////////////////////////////////////
class CityObjectState
{
public:
    /// Build a State and assign it a geometry
    CityObjectState(CityObject* geom);
    virtual ~CityObjectState();

    /// Get State id
    int getId() const;
    //std::string getStringId() const { std::stringstream ss; ss << "STATE" << m_id << m_name; return "";/*ss.str();*/ }

    /// Get State name
    virtual std::string getStringId() const;
    //std::string getGeomName() const { std::string a = m_geom?m_geom->getId():0; return getStringId()+a; }

    /// Get State geometry
    CityObject* getGeom();

    /// Get State geometry (const)
    const CityObject* getGeom() const;

    /// Get parent CityObject (the temporal object)
    const CityObject* getParent() const;

    /// Get an attribute for a specific date
    /// \param attribName Attribute name
    /// \param date Date wanted
    virtual std::string getAttribute(const std::string& attribName, const QDateTime& date) const;

    std::map<std::string, std::string> m_attributes;    ///< attributes map

    int m_id;                                           ///< State id (generated)
    std::string m_name;                                 ///< State name (generated if empty)
    CityObject* m_parent;                               ///< Parent object

private:
    CityObject* m_geom;                                 ///< CityObject describing State geometry
};
////////////////////////////////////////////////////////////////////////////////
/// \brief The DataSource class
///
/// Store temporal attributes for DynStates
///
/// m_dates and m_values are coupled. eg, value m_values[3] is related to date m_dates[3] for the attribute m_attribute
class DataSource
{
public:
    /// Build a datasource for a specific attribute
    /// \param attributeName Attribute related to this datasource
    DataSource(const std::string& attributeName);
    virtual ~DataSource();

    /// Get an attribute for a specific date
    /// \param date Date wanted
    virtual std::string getAttribute(const QDateTime& date) const;

   void dump() const;

//protected:
    std::string m_attribute;            ///< Attribute name

    std::vector<QDateTime> m_dates;     ///< Dates array
    std::vector<std::string> m_values;  ///< Values array
};
////////////////////////////////////////////////////////////////////////////////
/// \brief The DataSourceArray class
///
/// This is a datasource initialised from a block of text
///
/// format : date(format : yyyy/MM/dd-HH:mm:ss) | value (separator : |)
class DataSourceArray : public DataSource
{
public:
    /// Build a datasource for a specific attribute
    /// \param attributeName Attribute related to this datasource
    /// \param data Formated text containing data (format : date(format : yyyy/MM/dd-HH:mm:ss) | value (separator : |))
    DataSourceArray(const std::string& attributeName, const std::string& data);
    //virtual std::string getAttribute(const QDateTime& date) const override;

    /// Fill m_dates and m_values with data
    /// \param data Formated text containing data (format : date(format : yyyy/MM/dd-HH:mm:ss) | value (separator : |))
    void parse(const std::string& data);
};
////////////////////////////////////////////////////////////////////////////////
class DataSourceFile : public DataSource
{
public:
    /// Build a datasource for a specific attribute
    /// \param attributeName Attribute related to this datasource
    /// \param filePath Data file path
    ///
    /// File format :
    ///
    /// date1 (format : yyyy/MM/dd-HH:mm:ss)
    ///
    /// value1
    ///
    /// date2 (format : yyyy/MM/dd-HH:mm:ss)
    ///
    /// value2
    ///
    /// ...
    DataSourceFile(const std::string& attributeName, const std::string& filePath);
    //virtual std::string getAttribute(const QDateTime& date) const override;

    /// Fill m_dates and m_values with data
    /// \param filePath Data file path
    void parse(const std::string& filePath);

    std::string m_filePath; ///< Data file path
};
////////////////////////////////////////////////////////////////////////////////
/// \brief The CityObjectDynState class
///
/// Dynamic State having the possibility to store array of dated values for attributes
class CityObjectDynState : public CityObjectState
{
public:
    /// Build a State and assign it a geometry
    CityObjectDynState(CityObject* geom);
    virtual ~CityObjectDynState() override;

    /// Get State name
    virtual std::string getStringId() const;

    /// Add a datasource
    void addDataSource(DataSource* dsrc);

    /// Get an attribute for a specific date (will read datasources)
    /// \param attribName Attribute name
    /// \param date Date wanted
    virtual std::string getAttribute(const std::string& attribName, const QDateTime& date) const override;

private:
    std::map<std::string, DataSource*> m_attributes;    ///< attributes map
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __TEMPORALEXT_HPP__
