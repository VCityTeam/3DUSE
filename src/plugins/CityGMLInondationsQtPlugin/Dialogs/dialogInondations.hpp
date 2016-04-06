#ifndef DIALOG_INONDATIONS_H
#define DIALOG_INONDATIONS_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include <QFileDialog>
#include "export/exportCityGML.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class dialogInondations;
}
////////////////////////////////////////////////////////////////////////////////
class dialogInondations : public QDialog
{
    Q_OBJECT

public:
    explicit dialogInondations(QWidget *parent = 0);
    ~dialogInondations();

private:
    Ui::dialogInondations *ui;

public slots:
	void browseInputDirASCCut();
	void browseOutputDirASCCut();
	void cutASC();

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
#endif // DIALOG_INONDATIONS_H
