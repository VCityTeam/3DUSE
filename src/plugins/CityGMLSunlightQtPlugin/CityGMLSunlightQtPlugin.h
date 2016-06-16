#ifndef CITYGMLSunlightQTPLUGIN_H
#define CITYGMLSunlightQTPLUGIN_H

#include "../gui/pluginInterface.h"

#include <QStringList>
#include "Dialogs/DialogCityGMLSunlight1.h"

class CityGMLSunlightQtPlugin :
  public QObject,
  public Generic_PluginInterface
{
    Q_OBJECT
    Q_INTERFACES(Generic_PluginInterface)	
#if (WITH_QT5) // see at the end of .cxx for QT4
    Q_PLUGIN_METADATA(IID "CityGMLSunlightQtPlugin")
#endif
        
    QStringList Generic_plugins() const
	{
        return QStringList() << "CityGMLSunlightQtPlugin";
	}

    bool Generic_plugin(const QString &plugin)
    {
        init();
        return true;
    }

public slots:
    ///
    /// \brief ChangePolyColor Calls a function in osgScene which changes the color accordingly to the curent datetime.
    /// \param datetime current datetime
    ///
    void ChangePolyColor(QDateTime datetime);

    ///
    /// \brief startVisuAction Init visualisation (load sunlight informations and enables visu)
    /// \param filepaths filepaths of files to visualize sunlight for
    /// \param startDate Start date of visualization
    /// \param endDate end date of visualisation
    ///
    void startVisuAction(QStringList filepaths, QDateTime startDate, QDateTime endDate);

    ///
    /// \brief stopVisuAction Terminate visualisation
    ///
    void stopVisuAction();

private:
    ///
    /// \brief init Initialize plugin (create dialog, connect signals,..)
    /// \return 0
    ///
    int init(void);

    ///
    /// \brief loadFile loads sunlight information file into member m_sunlightInfo
    /// \param filepath Path of the file to load
    ///
    void loadFile(QString filepath);

    ///
    /// \brief loadSunlightFiles Load files selected for visualization
    /// \param filepaths paths of files to load
    /// \param startDate start date of the visualization
    /// \param endDate end date of the visualization
    ///
    void loadSunlightFiles(QStringList filepaths, QDateTime startDate, QDateTime endDate);


    ///
    /// \brief m_sunlightInfo map containing sunlight informations for files to visualize.
    ///
    ///  int: dateTime ; string : idPolygon ; bool : sunny
    ///
    std::map<int,std::map<std::string,bool>*> m_sunlightInfo;

    ///
    /// \brief m_visu true if visu start visu button clicked, false otherwise
    ///
    bool m_visu;
};

#endif // CITYGMLSunlightQTPLUGIN_H
