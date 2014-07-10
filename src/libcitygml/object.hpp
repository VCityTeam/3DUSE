#ifndef __CITYGML_OBJECT_HPP__
#define __CITYGML_OBJECT_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <string>
#include <map>
#include <ostream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
typedef std::map< std::string, std::string > AttributesMap;
///////////////////////////////////////////////////////////////////////////////
// Base object associated with an unique id and a set of attributes (key-value pairs)
class Object
{
    friend class CityGMLHandler;
    friend std::ostream& operator<<( std::ostream&, const Object & );
public:
    Object( const std::string& id );
    virtual ~Object( void );

    const std::string& getId( void ) const;

    std::string getAttribute( const std::string& name ) const;

    const AttributesMap& getAttributes() const;

    AttributesMap& getAttributes();

    //inline osg::ref_ptr<osg::Group> getOsgNode() { return m_osgNode; }
    //inline void setOsgNode(osg::ref_ptr<osg::Group> node) { m_osgNode = node; }

    //inline osg::Group* getOsgNode() { return m_osgNode; }
    //inline void setOsgNode(osg::Group* node) { m_osgNode->ref(); m_osgNode = node; }
    //inline void setOsgNode(osg::Group* node) { m_osgNode = node; }

    void setAttribute( const std::string& name, const std::string& value, bool forceOnExist = true );

protected:
    std::string _id;

    AttributesMap _attributes;

    //osg::ref_ptr<osg::Group> m_osgNode;
    //osg::Group* m_osgNode;
};
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream&, const citygml::Object& );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_OBJECT_HPP__
