#ifndef DialogCityGMLSunlight1_H
#define DialogCityGMLSunlight1_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include <QListWidget>
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
    bool ListContains(QListWidget* list, QString item);
    void AddItemsFromDirToList(QString dirpath);
    void AddCalculatedFilesToList(QString dirpath);

private slots:
    //Calculation
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
