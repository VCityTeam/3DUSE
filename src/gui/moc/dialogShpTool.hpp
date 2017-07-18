// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef DIALOGSHPTOOL_HPP
#define DIALOGSHPTOOL_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "libcitygml/URI.hpp"

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
