#ifndef __APPLICATION_HPP__
#define __APPLICATION_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "scene.hpp"
#include "dataprofile.hpp"
#include "settings.hpp"
#include "tools/log.hpp"
#include "algo.hpp"
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
    ~Application();

    /// \brief getScene Get the scene
    /// \return The scene
    Scene& getScene();

    /// \brief getScene Get the scene
    /// \return The scene
    const Scene& getScene() const;

    /// \brief getDataProfile Get the data profile
    /// \return The data profile
    DataProfile& getDataProfile();

    /// \brief getDataProfile Get the data profile
    /// \return The data profile
    const DataProfile& getDataProfile() const;

    /// \brief getController Get the controller
    /// \return The controller
    Controller* getController();

    /// \brief getController Get the controller
    /// \return The controller
    void setController(Controller* cont);

    /// \brief getAlgo Get algo class
    /// \return Ref on Aglo instance
    Algo& getAlgo();

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
    DataProfile m_dataprofile;          ///< data profile
    Settings m_settings;                ///< application settings
    Controller* m_controller;           ///< controller, needs to be allocated outside
    Log m_log;                          ///< Log manager
    Algo m_algo;                        ///< Algo class
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
