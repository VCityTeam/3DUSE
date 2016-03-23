// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef __OSGQTWIDGET_HPP__
#define __OSGQTWIDGET_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "osgQtWidget.hpp"
#include <osgViewer/CompositeViewer>
#include <osgQt/GraphicsWindowQt>
#include <QWidget>
#include <QtCore/QTimer>
#include <QTextBrowser>
/*#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osgGA/UFOManipulator>*/
#include "osgPicking.hpp"
////////////////////////////////////////////////////////////////////////////////
class osgQtWidget : public QWidget, public osgViewer::CompositeViewer
{
public:
   osgQtWidget(QWidget* parent=0);
   QWidget* addViewWidget(osgQt::GraphicsWindowQt* gw, QWidget* parent);
   osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name="", bool windowDecoration=false);
   virtual void paintEvent(QPaintEvent* event);
   void setSceneData(osg::Node* scene);

   void setPickHandler(PickHandler* pickhandler);

   //const std::string& getNodePicked() const;

   void centerCamera();

   PickHandler* getPickHandler();

   QWidget* getWidget();

   /// \brief setActive Enable or disable rendering update
   /// \param val true or false
   void setActive(bool val, int freq=1000);

protected:
   QTimer m_timer;

   /*osgGA::TrackballManipulator* m_trackballManipulator;
   osgGA::FlightManipulator* m_flightManipulator;
   osgGA::DriveManipulator* m_driveManipulator;
   osgGA::TerrainManipulator* m_terrainManipulator;
   osgGA::OrbitManipulator* m_orbitManipulator;
   osgGA::FirstPersonManipulator* m_firstPersonManipulator;
   osgGA::SphericalManipulator* m_SphericalManipulator;
   osgGA::UFOManipulator* m_UFOManipulator;*/

public:
   QWidget* m_widget;
   osgViewer::View* m_osgView;

   PickHandler* m_pickHandler;
   QTextBrowser* m_textBrowser;
};
////////////////////////////////////////////////////////////////////////////////
#endif // __OSGQTWIDGET_HPP__
