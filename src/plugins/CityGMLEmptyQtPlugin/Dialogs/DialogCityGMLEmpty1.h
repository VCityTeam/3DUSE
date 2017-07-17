// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DialogCityGMLEmpty1_H
#define DialogCityGMLEmpty1_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogCityGMLEmpty1;
}
////////////////////////////////////////////////////////////////////////////////
class DialogCityGMLEmpty1 : public QDialog
{
    Q_OBJECT

public:
    explicit DialogCityGMLEmpty1(QWidget *parent = 0);
    ~DialogCityGMLEmpty1();

    void setBoundingBox(unsigned int xmin, unsigned int ymin, unsigned int xmax, unsigned int ymax);
    void getBoundingBox(unsigned int& xmin, unsigned int& ymin, unsigned int& xmax, unsigned int& ymax);

private:
    Ui::DialogCityGMLEmpty1 *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DialogCityGMLEmpty1_H
