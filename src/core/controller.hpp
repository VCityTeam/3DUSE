// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <string>
#include "URI.hpp"
#include "application.hpp"
#include "ogrsf_frmts.h"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
    ////////////////////////////////////////////////////////////////////////////////
    /// \brief The Controller class
    ///
    /// Used to dispatch commands accross different layers (MVC use)
    class Controller
    {
    public:
        Controller();
        virtual ~Controller();

        // reset app
        virtual void reset();

        // generic nodes actions
        virtual void addNode(const URI& URI);
        virtual void deleteNode(const URI& URI);

        // layer
        virtual void addLayer(const std::string& name);
        virtual void deleteLayer(const vcity::URI& uri);
        virtual void setLayerName(const vcity::URI& uri, const std::string& name);

        // tile
        virtual void addTile(const vcity::URI& uriLayer, vcity::Tile& tile);
        virtual void deleteTile(const vcity::URI& uri);
        virtual void setTileName(const vcity::URI& uri, const std::string& name);

        // temporal
        virtual void addTag(const vcity::URI& uri, citygml::CityObjectTag* tag);
        virtual void addState(const vcity::URI& uri, citygml::CityObjectState* state);
        virtual void addDynState(const vcity::URI& uri, citygml::CityObjectDynState* state);

        // Shp
        virtual void addShpNode(const vcity::URI& uriLayer, OGRDataSource* poDS);

        // selection
        virtual void resetSelection();
        virtual bool addSelection(const vcity::URI& uri);
    };
    ////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __CONTROLLER_HPP__
