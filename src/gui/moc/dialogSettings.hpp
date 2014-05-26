#ifndef DIALOGSETTINGS_HPP
#define DIALOGSETTINGS_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "core/dataprofile.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogSettings;
}
////////////////////////////////////////////////////////////////////////////////
class DialogSettings : public QDialog
{
    Q_OBJECT

public:
    explicit DialogSettings(QWidget *parent = 0);
    ~DialogSettings();

    void doSettings();

private slots:
    void choosePathSlot();
    void chooseDataProfileSlot(int i);

private:
    void setFromDataProfile(const vcity::DataProfile& dp);
    void setDataProfileFromUI(vcity::DataProfile& dp);

    vcity::DataProfile m_tmpDP;

    Ui::DialogSettings *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGSETTINGS_HPP
