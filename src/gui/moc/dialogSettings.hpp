// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef DIALOGSETTINGS_HPP
#define DIALOGSETTINGS_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include <qdatetime.h>
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
	void setMinEndDate();

private:
    void setFromDataProfile(const vcity::DataProfile& dp);
    void setDataProfileFromUI(vcity::DataProfile& dp);

    vcity::DataProfile m_tmpDP;
    bool m_updateDataProfile;

    Ui::DialogSettings *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGSETTINGS_HPP
