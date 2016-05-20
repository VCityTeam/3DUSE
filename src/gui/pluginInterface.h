#ifndef PLUGININTERFACE_H
#define PLUGININTERFACE_H

#include <QtPlugin>

#include "moc/mainWindow.hpp"

/*!
* \brief Interfaces pour les plugins
* Ces interfaces seront utilisées pour les différents plugins.
*/

class Generic_PluginInterface
{
public:

    virtual ~Generic_PluginInterface() {}

    virtual QStringList Generic_plugins() const = 0;
    virtual bool Generic_plugin(const QString &plugin) = 0;

    virtual void init(MainWindow* mainWindow)
    {
        this->mw = mainWindow;
    }

protected:
    MainWindow* mw;	//!< the MainWindow pointer
};

Q_DECLARE_INTERFACE(Generic_PluginInterface,
    "fr.liris.3DUSE.Generic_PluginInterface/1.0")

#endif // PLUGININTERFACE_H
