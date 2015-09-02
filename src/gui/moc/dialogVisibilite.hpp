// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef DIALOGVISIBILITE_HPP
#define DIALOGVISIBILITE_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "visibilite/Export.hpp"

////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogVisibilite;
}

class MainWindow;

////////////////////////////////////////////////////////////////////////////////
class DialogVisibilite : public QDialog
{
    Q_OBJECT

public:
    explicit DialogVisibilite(QWidget *parent, MainWindow* mainwindow);
    ~DialogVisibilite();

private:
    Ui::DialogVisibilite *ui;
	MainWindow* mainwindow;

	void SetupEmblematicViewExportParameter();

	osg::ref_ptr<osg::Camera> SetupRenderingCamera();

private slots:
	void DirButtonClicked();
	void GetCamParam();
	void SetCamParam();
	void BasicMultiTile();
	void BasicMonoTile();
	void BasicPanorama();
	void CascadeMultiTile();
	void CascadeMonoTile();
	void CascadePanorama();
	void ResetCategory();
	void ToolAlignementTree();
	void ToolLidarToGML();
	void ToolShpExtrusion();
	void ToolFlatRoof();
	void ToolAABBReconstruction();
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGEDITTILE_HPP
