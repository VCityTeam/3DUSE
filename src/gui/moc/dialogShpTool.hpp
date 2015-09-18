// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef DIALOGSHPTOOL_HPP
#define DIALOGSHPTOOL_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "core/URI.hpp"

#include "src/gui/osg/osgGDAL.hpp"
#include <osg/MatrixTransform>
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogShpTool;
}

class MainWindow;

////////////////////////////////////////////////////////////////////////////////
class DialogShpTool : public QDialog
{
    Q_OBJECT

public:
    explicit DialogShpTool(QWidget *parent, MainWindow* mainwindow);
    ~DialogShpTool();

	void Setup(const vcity::URI& uri);

private:
    Ui::DialogShpTool *ui;
	MainWindow* mainwindow;
	osg::ref_ptr<osg::Node> node;

	osg::MatrixTransform* GetMatrixTransform();

private slots:
	void HeightValueChanged(double d);
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGEDITTILE_HPP
