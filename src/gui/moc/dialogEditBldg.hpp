#ifndef DIALOGEDITBLDG_H
#define DIALOGEDITBLDG_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
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
