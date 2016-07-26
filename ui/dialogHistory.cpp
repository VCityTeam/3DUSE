#include "dialogHistory.h"
#include "ui_dialogHistory.h"

dialogHistory::dialogHistory(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::dialogHistory)
{
    ui->setupUi(this);
}

dialogHistory::~dialogHistory()
{
    delete ui;
}
