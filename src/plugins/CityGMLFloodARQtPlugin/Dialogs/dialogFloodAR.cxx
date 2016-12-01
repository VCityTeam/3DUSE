#include "dialogFloodAR.hpp"
#include "ui_dialogFloodAR.h"
#include <QMessageBox>

#include "../FloodARTools.hpp"
#include "AABB.hpp"

////////////////////////////////////////////////////////////////////////////////
dialogFloodAR::dialogFloodAR( QWidget *parent ) :
   QDialog( parent ),
   ui( new Ui::dialogFloodAR )
{
   ui->setupUi( this );
   connect( ui->btn_exit, SIGNAL( clicked() ), this, SLOT( close() ) );
   connect( ui->btn_wkingDir, SIGNAL( clicked() ), this, SLOT( browseWorkingDirectory() ) );
   connect( ui->btn_ASCcut_in, SIGNAL( clicked() ), this, SLOT( browseInputASCCut() ) );
   connect( ui->btn_ASCcut_exec, SIGNAL( clicked() ), this, SLOT( cutASC() ) );
   connect( ui->btn_ASCtoWater_exec, SIGNAL( clicked() ), this, SLOT( ASCtoWater() ) );
   connect( ui->btn_ASCtoTerrain_exec, SIGNAL( clicked() ), this, SLOT( ASCtoTerrain() ) );
   connect( ui->btn_ShpExt_buildAABB, SIGNAL( clicked() ), this, SLOT( buildBoundingBoxes() ) );
   connect( ui->btn_ShpExt_exec, SIGNAL( clicked() ), this, SLOT( ShpExtrusion() ) );
   connect( ui->btn_texCut_in, SIGNAL( clicked() ), this, SLOT( browseInputTextureCut() ) );
   connect( ui->btn_texCut_exec, SIGNAL( clicked() ), this, SLOT( textureCut() ) );
   connect( ui->btn_shpCut_in, SIGNAL( clicked() ), this, SLOT( browseInputSHPCut() ) );
   connect( ui->btn_shpCut_exec, SIGNAL( clicked() ), this, SLOT( SHPCut() ) );

   // init ASCCut radio group
   ui->radioBtn_ASCCut_terrain->setChecked( true );
}
////////////////////////////////////////////////////////////////////////////////
dialogFloodAR::~dialogFloodAR()
{
   delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseWorkingDirectory()
{
   QString path = QFileDialog::getExistingDirectory( this, "Select working directory" );
   ui->lineEdit_wkingDir->setText( path );
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInputASCCut()
{
   QStringList filenames = QFileDialog::getOpenFileNames( this, "Select ASC source file", "", "ASC files (*.asc)" );
   ui->lineEdit_ASCcut_src->setText( filenames.join( ";" ) );
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::cutASC()
{
   QStringList files( ui->lineEdit_ASCcut_src->text().split( ";" ) );
   QDir dir( ui->lineEdit_wkingDir->text() );
   int tileSizeX = ui->spinBox_tileSize_x->value();
   int tileSizeY = ui->spinBox_tileSize_y->value();
   bool isTerrain = ui->radioBtn_ASCCut_terrain->isChecked();
   if ( !dir.exists() || ui->lineEdit_wkingDir->text().toStdString() == "" )
   {
      QMessageBox msgBox;
      msgBox.setText( "Working directory not found!" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }
   if ( !( tileSizeX > 0 && tileSizeY > 0 ) )
   {
      QMessageBox msgBox;
      msgBox.setText( "Invalid Tile Size" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }
   for ( QFileInfo file : files )
   {
      if ( !file.exists() )
      {
         QMessageBox msgBox;
         msgBox.setText( ( "Input file not found! (" + file.absoluteFilePath().toStdString() + ")" ).c_str() );
         msgBox.setIcon( QMessageBox::Critical );
         msgBox.exec();
         continue;
      }
      QString ext = file.suffix().toLower();
      if ( ext == "asc" )
      {
         std::cout << "Tiling file " << file.absoluteFilePath().toStdString() << std::endl;
         FloodAR::cutASC( file.absoluteFilePath().toStdString(), dir.absolutePath().toStdString(), tileSizeX, tileSizeY, isTerrain );
      }
   }
   std::cout << "Job done!" << std::endl;
   QMessageBox msgBox;
   msgBox.setText( "Tiling finished!" );
   msgBox.setIcon( QMessageBox::Information );
   msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInputTextureCut()
{
   QString filename = QFileDialog::getOpenFileName( this, "Select texture file", "", "" );
   ui->lineEdit_texCut_src->setText( filename );
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::textureCut()
{
   QFileInfo file( ui->lineEdit_texCut_src->text() );
   QDir wkDir( ui->lineEdit_wkingDir->text() );
   int tileSizeX = ui->spinBox_txTileSize_x->value();
   int tileSizeY = ui->spinBox_txTileSize_y->value();
   if ( !file.exists() )
   {
      QMessageBox msgBox;
      msgBox.setText( "Input file not found!" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }
   if ( !( tileSizeX > 0 && tileSizeY > 0 ) )
   {
      QMessageBox msgBox;
      msgBox.setText( "Invalid Tile Size" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }
   if ( !wkDir.exists() || ui->lineEdit_wkingDir->text().toStdString() == "" )
   {
      QMessageBox msgBox;
      msgBox.setText( "Working directory not found!" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }

   FloodAR::cutPicture( file.absoluteFilePath().toStdString(), ui->lineEdit_wkingDir->text().toStdString(), tileSizeX, tileSizeY );

   std::cout << "Job done!" << std::endl;
   QMessageBox msgBox;
   msgBox.setText( "Tiling finished!" );
   msgBox.setIcon( QMessageBox::Information );
   msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::browseInputSHPCut()
{
   QString filename = QFileDialog::getOpenFileName( this, "Select SHP file", "", "SHP files (*.shp)" );
   ui->lineEdit_shpCut_src->setText( filename );
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::SHPCut()
{
   QFileInfo file( ui->lineEdit_shpCut_src->text() );
   QDir wkDir( ui->lineEdit_wkingDir->text() );
   int tileSizeX = ui->spinBox_shpTileSize_x->value();
   int tileSizeY = ui->spinBox_shpTileSize_y->value();

   if ( !file.exists() )
   {
      QMessageBox msgBox;
      msgBox.setText( "Input file not found!" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }
   if ( !wkDir.exists() || ui->lineEdit_wkingDir->text().toStdString() == "" )
   {
      QMessageBox msgBox;
      msgBox.setText( "Working directory not found!" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }
   if ( !( tileSizeX > 0 && tileSizeY > 0 ) )
   {
      QMessageBox msgBox;
      msgBox.setText( "Invalid Tile Size" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }

   QMessageBox confirmBox;
   confirmBox.setInformativeText( "This operation may take time and use a lot of memory. It is recommanded that you close all other applications and don't use your computer during its execution." );
   confirmBox.setText( "Do you wish to continue?" );
   confirmBox.setStandardButtons( QMessageBox::Ok | QMessageBox::Cancel );
   confirmBox.setIcon( QMessageBox::Warning );
   int ret = confirmBox.exec();
   if ( ret != QMessageBox::Ok ) return;

   FloodAR::CutShapeFile( ui->lineEdit_wkingDir->text().toStdString(), tileSizeX, tileSizeY, file.absoluteFilePath().toStdString() );

   std::cout << "Job done!" << std::endl;
   QMessageBox msgBox;
   msgBox.setText( "Tiling finished!" );
   msgBox.setIcon( QMessageBox::Information );
   msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::ASCtoWater()
{
   float prec = ui->dbSpinBox_ASCtoWater_prec->value();
   QDateTime creaDate = ui->dtEdit_creationDate->dateTime();
   QDir wkDir( ui->lineEdit_wkingDir->text() );
   if ( !wkDir.exists() )
   {
      QMessageBox msgBox;
      msgBox.setText( "Working directory not found!" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }

   FloodAR::ASCtoWaterAuto( wkDir.absolutePath().toStdString(), prec, creaDate.toString( Qt::ISODate ).toStdString() );

   std::cout << "Job done!" << std::endl;
   QMessageBox msgBox;
   msgBox.setText( "Conversion finished!" );
   msgBox.setIcon( QMessageBox::Information );
   msgBox.exec();
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::ASCtoTerrain()
{
   std::string dir = ui->lineEdit_wkingDir->text().toStdString();
   QDir wkDir( dir.c_str() );
   if ( !wkDir.exists() || ui->lineEdit_wkingDir->text().toStdString() == "" )
   {
      QMessageBox msgBox;
      msgBox.setText( "Working directory not found!" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }
   FloodAR::ASCtoTerrain( dir );

   std::cout << "Job done!" << std::endl;
   QMessageBox msgBox;
   msgBox.setText( "Conversion finished!" );
   msgBox.setIcon( QMessageBox::Information );
   msgBox.exec();
}
//////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::buildBoundingBoxes()
{
   std::string dir = ui->lineEdit_wkingDir->text().toStdString();
   BuildLayersAABBs( dir + "/" );
}
////////////////////////////////////////////////////////////////////////////////
void dialogFloodAR::ShpExtrusion()
{
   std::string dir = ui->lineEdit_wkingDir->text().toStdString();
   QDir wkDir( dir.c_str() );
   if ( !wkDir.exists() || ui->lineEdit_wkingDir->text().toStdString() == "" )
   {
      QMessageBox msgBox;
      msgBox.setText( "Working directory not found!" );
      msgBox.setIcon( QMessageBox::Critical );
      msgBox.exec();
      return;
   }

   FloodAR::ShapeExtrusion( dir );

   std::cout << "Job done!" << std::endl;
   QMessageBox msgBox;
   msgBox.setText( "Conversion finished!" );
   msgBox.setIcon( QMessageBox::Information );
   msgBox.exec();
}
