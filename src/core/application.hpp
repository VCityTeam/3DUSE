#ifndef __APPLICATION_HPP__
#define __APPLICATION_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "scene.hpp"
#include "dataprofile.hpp"
#include "settings.hpp"
#include "tools/log.hpp"
#include "algo.hpp"
#include "algo2.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
class Controller;
////////////////////////////////////////////////////////////////////////////////
/// \brief The Application class
/// Stores Common application data
class Application
{
public:
    Application();
    virtual ~Application();

    /// \brief getScene Get the scene
    /// \return The scene
    Scene& getScene();

    /// \brief getScene Get the scene
    /// \return The scene
    const Scene& getScene() const;

    /// \brief getSettings Get app settings
    /// \return Settings class instance
    Settings& getSettings();

    /// \brief getSettings Get app settings
    /// \return Settings class instance
    const Settings& getSettings() const;

    /// \brief getController Get the controller
    /// \return The controller
    Controller* getController();

    /// \brief getController Get the controller
    /// \return The controller
    void setController(Controller* cont);

    /// \brief getAlgo Get algo class
    /// \return Ref on Aglo instance
    Algo& getAlgo();

    /// \brief getAlgo2 Get algo2 class
    /// \return Ref on Aglo2 instance
    Algo2& getAlgo2();

    /// \brief getSelectedNodes Get selected nodes (in treeview or in osg)
    /// \return Array of URI
    const std::vector<URI>& getSelectedNodes() const;

    /// \brief setSelectedNodes Set selected nodes
    /// \param uris Array of uri ponting to selected nodes
    void setSelectedNodes(const std::vector<URI>& uris);

    /// \brief addSelectedNode Add a selected node
    /// \param uri URI pointing to the selected node to add
    bool addSelectedNode(const URI& uri);

    /// \brief resetSelectedNode Rest seleceted nodes list
    void resetSelectedNodes();

protected:
    Scene m_scene;                      ///< scene tree : citygml + shape + ...
    Settings m_settings;                ///< application settings
    Controller* m_controller;           ///< controller, needs to be allocated outside
    Log m_log;                          ///< Log manager
    Algo m_algo;                        ///< Algo class
    Algo2 m_algo2;                        ///< Algo class
    std::vector<URI> m_selectedNodes;   ///< Selected nodes
};
////////////////////////////////////////////////////////////////////////////////
Application& app();
Log& log();
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#include "controller.hpp"
////////////////////////////////////////////////////////////////////////////////
#endif // __APPLICATION_HPP__
