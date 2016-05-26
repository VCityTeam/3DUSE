// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGCITYGML_HPP__
#define __OSGCITYGML_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Texture2D>
#include "citygml.hpp"
////////////////////////////////////////////////////////////////////////////////
class ReaderOsgCityGML
{
public:
    class Settings
    {
    public:
        Settings();

    public:
        citygml::ParserParams _params;
        bool _printNames;
        bool _first;
        bool _useMaxLODOnly;
        bool _recursive;
        bool m_useTextures;
        osg::Vec3 _origin;
        std::map< std::string, osg::Texture2D* > _textureMap;
        std::string m_filepath;
    };

    ReaderOsgCityGML(const std::string& filepath);
    osg::ref_ptr<osg::Node> readNode(const citygml::CityObject* citygml);

    static unsigned int getHighestLodForObject(const citygml::CityObject * object);

    //private:
    osg::ref_ptr<osg::Node> readCity(const citygml::CityModel*);
    osg::ref_ptr<osg::Group> createCityObject(const citygml::CityObject*, unsigned int minimumLODToConsider = 0);

public:
    Settings m_settings;
};
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGCITYGML_HPP__
