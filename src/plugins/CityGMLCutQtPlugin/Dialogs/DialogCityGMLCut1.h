#ifndef DialogCityGMLCut1_H
#define DialogCityGMLCut1_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogCityGMLCut1;
}
////////////////////////////////////////////////////////////////////////////////
class DialogCityGMLCut1 : public QDialog
{
    Q_OBJECT

public:
    explicit DialogCityGMLCut1(QWidget *parent = 0);
    ~DialogCityGMLCut1();

	void setGMLFiles(QString IN, QString OUT);
    void getGMLFiles(QString& IN, QString& OUT);

    void setBoundingBox(unsigned int xmin, unsigned int ymin, unsigned int xmax, unsigned int ymax);
    void getBoundingBox(unsigned int& xmin, unsigned int& ymin, unsigned int& xmax, unsigned int& ymax);

	void setVerbose(bool check);
    bool getVerbose();

private:
    Ui::DialogCityGMLCut1 *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DialogCityGMLCut1_H
