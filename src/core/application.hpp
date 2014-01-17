#ifndef __APPLICATION_HPP__
#define __APPLICATION_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "scene.hpp"
#include "dataprofile.hpp"
#include "settings.hpp"
#include "controller.hpp"
#include "tools/log.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The Application class
/// Stores Common application data
class Application
{
public:
    Application();

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

protected:
    Scene m_scene;              ///< scene tree : citygml + shape + ...
    DataProfile m_dataprofile;  ///< data profile
    Settings m_settings;        ///< application settings
    Controller* m_controller;   ///< controller, needs to be allocated outside
    Log m_log;                  ///< Log manager
};
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __APPLICATION_HPP__
