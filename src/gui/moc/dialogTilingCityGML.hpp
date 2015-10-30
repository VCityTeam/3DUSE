#ifndef DIALOGTILINGCITYGML_HPP
#define DIALOGTILINGCITYGML_HPP

#include <QDialog>

namespace Ui {
class DialogTilingCityGML;
}

class DialogTilingCityGML : public QDialog
{
    Q_OBJECT

private slots:
    void chooseCityGMLPathSlot();
    void chooseOutputPathSlot();
    void TilingCityGMLSlot();

public:
    explicit DialogTilingCityGML(QWidget *parent = 0);
    ~DialogTilingCityGML();

private:
    Ui::DialogTilingCityGML *ui;
};

#endif // DIALOGTILINGCITYGML_HPP
