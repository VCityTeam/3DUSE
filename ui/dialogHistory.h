#ifndef DIALOGHISTORY_H
#define DIALOGHISTORY_H

#include <QDialog>

namespace Ui {
class dialogHistory;
}

class dialogHistory : public QDialog
{
    Q_OBJECT

public:
    explicit dialogHistory(QWidget *parent = 0);
    ~dialogHistory();

private:
    Ui::dialogHistory *ui;
};

#endif // DIALOGHISTORY_H
