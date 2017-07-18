// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
