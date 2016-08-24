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
