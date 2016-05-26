#ifndef DIALOG_FloodAR_H
#define DIALOG_FloodAR_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include <QFileDialog>
#include "export/exportCityGML.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
    class dialogFloodAR;
}
////////////////////////////////////////////////////////////////////////////////
class dialogFloodAR : public QDialog
{
    Q_OBJECT

public:
    explicit dialogFloodAR(QWidget *parent = 0);
    ~dialogFloodAR();

private:
    Ui::dialogFloodAR *ui;

    public slots:

    void browseWorkingDirectory();

    void browseInputASCCut();
    void cutASC();

    void browseInputTextureCut();
    void textureCut();

    void browseInputSHPCut();
    void SHPCut();

    void ASCtoWater();

    void ASCtoTerrain();

    void buildBoundingBoxes();
    void ShpExtrusion();

};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOG_FloodAR_H
