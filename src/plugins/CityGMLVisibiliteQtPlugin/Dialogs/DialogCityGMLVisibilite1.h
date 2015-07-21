#ifndef DialogCityGMLVisibilite1_H
#define DialogCityGMLVisibilite1_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogCityGMLVisibilite1;
}
////////////////////////////////////////////////////////////////////////////////
class DialogCityGMLVisibilite1 : public QDialog
{
    Q_OBJECT

public:
    explicit DialogCityGMLVisibilite1(QWidget *parent = 0);
    ~DialogCityGMLVisibilite1();

    void setBoundingBox(unsigned int xmin, unsigned int ymin, unsigned int xmax, unsigned int ymax);
    void getBoundingBox(unsigned int& xmin, unsigned int& ymin, unsigned int& xmax, unsigned int& ymax);

private:
    Ui::DialogCityGMLVisibilite1 *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DialogCityGMLVisibilite1_H
