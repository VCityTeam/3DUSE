// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "moc/dialogTilingCityGML.hpp"
#include "ui_dialogTilingCityGML.h"
#include "gui/applicationGui.hpp"
#include "moc/mainWindow.hpp"
#include <QFileDialog>
////////////////////////////////////////////////////////////////////////////////
DialogTilingCityGML::DialogTilingCityGML( QWidget *parent ) :
   QDialog( parent ),
   ui( new Ui::DialogTilingCityGML )
{
   ui->setupUi( this );

   connect( ui->pushButton, SIGNAL( clicked() ), this, SLOT( chooseCityGMLPathSlot() ) );
   connect( ui->pushButton_2, SIGNAL( clicked() ), this, SLOT( chooseOutputPathSlot() ) );
   connect( ui->buttonBox, SIGNAL( accepted() ), this, SLOT( TilingCityGMLSlot() ) );
}
////////////////////////////////////////////////////////////////////////////////
DialogTilingCityGML::~DialogTilingCityGML()
{
   delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogTilingCityGML::chooseCityGMLPathSlot()
{
   QString path = QFileDialog::getExistingDirectory( this, "Choose CityGML files path." );
   ui->lineEditDataPath->setText( path );
}
////////////////////////////////////////////////////////////////////////////////
void DialogTilingCityGML::chooseOutputPathSlot()
{
   QString path = QFileDialog::getExistingDirectory( this, "Choose Output path." );
   ui->lineEditDataPath_2->setText( path );
}
////////////////////////////////////////////////////////////////////////////////
void DialogTilingCityGML::TilingCityGMLSlot()
{
   appGui().getMainWindow()->TilingCityGML( ui->lineEditDataPath->text(), ui->lineEditDataPath_2->text().toStdString(), ui->lineTileSizeX->text().toInt(), ui->lineTileSizeY->text().toInt(), ui->le_Suffix->text().toStdString(), ui->rb_Assign->isChecked() );
}
////////////////////////////////////////////////////////////////////////////////
