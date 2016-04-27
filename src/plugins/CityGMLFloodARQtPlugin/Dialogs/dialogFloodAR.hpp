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
	void browseInputDirASCCut();
	void browseOutputDirASCCut();
	void cutASC();

	void browseInputTextureCut();
	void textureCut();

	void browseInputASCtoWater();
	void enablePolygonsParams(int);
	void enableTemporalParams(int);
	void ASCtoWater();

	void browseInput1ASCtoTerrain();
	void browseInput2ASCtoTerrain();
	void browseTextureASCtoTerrain();
	void enableASCFusion(int);
	void enableTextures(int);
	void ASCtoTerrain();

	void browseInputDirShpExt();
	void browseInputShpExt();
	void ShpExtrusion();

};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOG_FloodAR_H
