// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DialogCityGMLSunlight1_H
#define DialogCityGMLSunlight1_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include <QListWidget>
#include <QDateTime>
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogCityGMLSunlight1;
}
////////////////////////////////////////////////////////////////////////////////
class DialogCityGMLSunlight1 : public QDialog
{
    Q_OBJECT

public:
    explicit DialogCityGMLSunlight1(QWidget *parent = 0);
    ~DialogCityGMLSunlight1();

private:
    ///
    /// \brief ListContains Check if a given item is in a list (QListWidget)
    /// \param list List of items
    /// \param item item to find
    /// \return true if item is in list, false either
    ///
    bool ListContains(QListWidget* list, QString item);

    ///
    /// \brief AddItemsFromDirToList Get all filenames from a specified directory and add them to a QListWidget (Non selected files list in computation tab)
    /// \param dirpath Full path to directory to scan
    ///
    void AddItemsFromDirToList(QString dirpath);

    ///
    /// \brief AddCalculatedFilesToList Get name of all files for which sunlight has been computed and add them to a QListWidget (Non selected files list in visualization tab)
    /// \param dirpath Full path to directory to scan
    ///
    void AddComputedFilesToList(QString dirpath);

signals:
    ///
    /// \brief startVisu signal sent when startVisu Button is clicked and trapped by CityGMLSunlightQTPlugin
    /// \param filepaths paths of files to visualize
    /// \param startDate Start date of visualization
    /// \param endDate End date of visualization
    ///
    void startVisu(QStringList filepaths, QDateTime startDate, QDateTime endDate);

    ///
    /// \brief stopVisu signal sent when stopVisu Button is clicked and trapped by CityGMLSunlightQTPlugin
    ///
    void stopVisu();

private slots:
    //Computation
    void DirFilesButtonClicked();
    void AddFileButtonClicked();
    void RemoveFileButtonClicked();
    void AddAllFilesButtonClicked();
    void ClearAllFilesButtonClicked();
    void AnnualSunPathButtonClicked();
    void OutpoutDirButtonClicked();
    void CreateSunlightFilesButtonClicked();
    void StartDateChanged();
    void EndDateChanged();
    //Visu
    void InputDirButtonClicked();
    void VisuAddFileButtonClicked();
    void VisuRemoveFileButtonClicked();
    void VisuAddAllFilesButtonClicked();
    void VisuClearAllFilesButtonClicked();
    void VisuDirFilesButtonClicked();
    void StartVisuButtonClicked();
    void StopVisuButtonClicked();

private:
    Ui::DialogCityGMLSunlight1 *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DialogCityGMLSunlight1_H
