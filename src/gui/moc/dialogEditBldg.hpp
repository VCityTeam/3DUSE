// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef DIALOGEDITBLDG_H
#define DIALOGEDITBLDG_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "libcitygml/URI.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
    class DialogEditBldg;
}
////////////////////////////////////////////////////////////////////////////////
class DialogEditBldg : public QDialog
{
    Q_OBJECT

public:
    explicit DialogEditBldg(QWidget *parent = 0);
    ~DialogEditBldg();

    void edit(const vcity::URI& uri);

    void setName(const QString& str);
    QString getName() const;

    //void setEnvelope(double a, double b, double c, double d);

    void setOffset(double x, double y);
    void getOffset(double& x, double& y);

private:
    Ui::DialogEditBldg *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGEDITBLDG_H
