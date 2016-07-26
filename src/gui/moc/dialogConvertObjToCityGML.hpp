#ifndef DIALOGCONVERTOBJTOCITYGML_HPP
#define DIALOGCONVERTOBJTOCITYGML_HPP

#include <QDialog>

namespace Ui {
    class DialogConvertObjToCityGML;
}

class DialogConvertObjToCityGML : public QDialog
{
    Q_OBJECT

        private slots:
    void chooseCityGMLPathSlot();
    void chooseOutputPathSlot();
    void ConvertObjToCityGMLSlot();

private:
    QStringList OBJfilenames;
    double offsetX;
    double offsetY;

public:
    explicit DialogConvertObjToCityGML(double OffsetX, double OffsetY, QWidget *parent = 0);
    ~DialogConvertObjToCityGML();

private:
    Ui::DialogConvertObjToCityGML *ui;
};

#endif // DIALOGCONVERTOBJTOCITYGML_HPP
