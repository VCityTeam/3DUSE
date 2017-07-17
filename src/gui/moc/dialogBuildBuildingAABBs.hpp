// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DIALOGBUILDBUILDINGAABBS_HPP
#define DIALOGBUILDBUILDINGAABBS_HPP

#include <QDialog>

namespace Ui {
class DialogBuildBuildingAABBs;
}

class DialogBuildBuildingAABBs : public QDialog
{
    Q_OBJECT

public:
    explicit DialogBuildBuildingAABBs(QWidget *parent = 0);
    ~DialogBuildBuildingAABBs();

private:
    Ui::DialogBuildBuildingAABBs *ui;

private slots:
    void DirFilesButtonClicked();
    void slotBuildBuildingAABBs();
};

#endif // DIALOGBUILDBUILDINGAABBS_HPP
