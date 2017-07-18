// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "moc/dialogBuildBuildingAABBs.hpp"
#include "ui_dialogBuildBuildingAABBs.h"

#include <QFileDialog>

#include "libcitygml/utils/AABB.hpp"

DialogBuildBuildingAABBs::DialogBuildBuildingAABBs(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogBuildBuildingAABBs)
{
    ui->setupUi(this);

    connect(ui->B_FilesDir,SIGNAL(clicked()),this,SLOT(DirFilesButtonClicked()));
    connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(slotBuildBuildingAABBs()));
}

DialogBuildBuildingAABBs::~DialogBuildBuildingAABBs()
{
    delete ui;
}

void DialogBuildBuildingAABBs::DirFilesButtonClicked()
{
    QString dirpath = QFileDialog::getExistingDirectory(nullptr,"Choose Building Files Directory");

    //Add path to directory in LineEdit
    ui->LE_FilesDir->setText(dirpath);
}

void DialogBuildBuildingAABBs::slotBuildBuildingAABBs()
{
    BuildBuildingAABBs(ui->LE_FilesDir->text());
}
