#ifndef DialogCityGMLSunlight1_H
#define DialogCityGMLSunlight1_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
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
    void AddItemsFromDirToList(QString dirpath);

private slots:
    void DirFilesButtonClicked();
    void AddFileButtonClicked();
    void RemoveFileButtonClicked();
    void AddAllFilesButtonClicked();
    void ClearAllFilesButtonClicked();
    void AnnualSunPathButtonClicked();
    void CreateSunlightFilesButtonClicked();


private:
    Ui::DialogCityGMLSunlight1 *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DialogCityGMLSunlight1_H
