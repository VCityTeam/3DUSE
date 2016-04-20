// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "osgQtWidget.hpp"
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osgGA/UFOManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osg/io_utils>
//#include <QtWidgets/QGridLayout>
#include <QResizeEvent>
#include <iostream>
#include <sstream>
#include "gui/moc/mainWindow.hpp"
////////////////////////////////////////////////////////////////////////////////
class MyFirstPersonManipulator : public osgGA::FirstPersonManipulator
{
public:
    void moveForward( const double distance )
    {
        osgGA::FirstPersonManipulator::moveForward(distance);
    }

    void moveRight( const double distance )
    {
        osgGA::FirstPersonManipulator::moveRight(distance);
    }

    void moveUp( const double distance )
    {
        osgGA::FirstPersonManipulator::moveUp(distance);
    }
};
////////////////////////////////////////////////////////////////////////////////
osg::Matrixd mat;
////////////////////////////////////////////////////////////////////////////////
class CameraHandler : public osgGA::GUIEventHandler
{
public:
    CameraHandler()
        : m_mod(false), m_speed(50.0f)
    {
    }

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        osgViewer::View* viewer = dynamic_cast<osgViewer::View*>(&aa);
        if (!viewer) return false;

        switch(ea.getEventType())
        {
        /*case(osgGA::GUIEventAdapter::PUSH):
        {
            if((ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) != 0)
            {
                m_mod = true;
                std::cout << "mod key on" << std::endl;
            }
            else
            {
                m_mod = false;
                std::cout << "mod key off" << std::endl;
            }
            return true;
            break;
        }*/
        case(osgGA::GUIEventAdapter::DRAG):
        {
            appGui().getMainWindow()->m_osgView->setActive(true);
            break;
        }
        case(osgGA::GUIEventAdapter::RELEASE):
        {
            appGui().getMainWindow()->m_osgView->setActive(false, 100);
            break;
        }
        case(osgGA::GUIEventAdapter::DOUBLECLICK):
        {
            if(appGui().getSelectedNodes().size() > 0)
            {
                appGui().getOsgScene()->centerOn(appGui().getSelectedNodes()[0]);
            }
            break;
        }
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            if((ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) != 0)
            {
                m_mod = true;
                //std::cout << "mod key on" << std::endl;
            }
            else
            {
                m_mod = false;
                //std::cout << "mod key off" << std::endl;
            }

            // check if we are using first person camera
            osgGA::KeySwitchMatrixManipulator* ksm = static_cast<osgGA::KeySwitchMatrixManipulator*>(viewer->getCameraManipulator());
            MyFirstPersonManipulator* fpsCam = dynamic_cast<MyFirstPersonManipulator*>(ksm->getCurrentMatrixManipulator());
            if(fpsCam != NULL)
            {
                switch(ea.getKey())
                {
                case osgGA::GUIEventAdapter::KEY_Up:
                {
                    if(m_mod)
                        fpsCam->moveUp(m_speed);
                    else
                        fpsCam->moveForward(m_speed);
                    break;
                }
                case osgGA::GUIEventAdapter::KEY_Down:
                {
                    if(m_mod)
                        fpsCam->moveUp(-m_speed);
                    else
                        fpsCam->moveForward(-m_speed);
                    break;
                }
                case osgGA::GUIEventAdapter::KEY_Left:
                {
                    if(m_mod)
                        m_speed -= 5.0f;
                    else
                        fpsCam->moveRight(-m_speed);
                    break;
                }
                case osgGA::GUIEventAdapter::KEY_Right:
                {
                    if(m_mod)
                        m_speed += 5.0f;
                    else
                        fpsCam->moveRight(m_speed);
                    break;
                }
                case osgGA::GUIEventAdapter::KEY_I:
                {
                    mat = fpsCam->getMatrix();
                    break;
                }
                case osgGA::GUIEventAdapter::KEY_O:
                {
                    fpsCam->setByMatrix(mat);
                    break;
                }
                default:
                    break;
                }
            }
        }
            break;
        default:
            break;
        }
        return false;
    }

private:
    bool m_mod;
    float m_speed;
};
////////////////////////////////////////////////////////////////////////////////
osgQtWidget::osgQtWidget(QWidget* parent)
    : QWidget(parent)//, m_osgScene(new osg::Group())
{
#ifdef Q_WS_X11
   setThreadingModel(osgViewer::ViewerBase::CullThreadPerCameraDrawThreadPerContext);
#else
	#if (WITH_QT5)
		setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
	#else
		setThreadingModel(osgViewer::ViewerBase::CullThreadPerCameraDrawThreadPerContext);
	#endif
#endif
   m_widget = addViewWidget(createGraphicsWindow(0,0,100,100), parent);
   connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
   m_timer.start(15);

   // reduce fps when app is idle
   setActive(false, 100);
}
////////////////////////////////////////////////////////////////////////////////
QWidget* osgQtWidget::addViewWidget(osgQt::GraphicsWindowQt* gw, QWidget* /*parent*/)
{
   osgViewer::View* view = new osgViewer::View;
   m_osgView = view;
   addView(view);

   osg::Camera* camera = view->getCamera();
   camera->setGraphicsContext(gw);

   const osg::GraphicsContext::Traits* traits = gw->getTraits();

   camera->setClearColor( osg::Vec4(0.46, 0.7, 0.99, 1.0) );
   //camera->setClearColor( osg::Vec4(1.0, 1.0, 1.0, 1.0) );
   camera->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
   camera->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );

   view->addEventHandler( new osgViewer::StatsHandler );
   //view->setCameraManipulator( new osgGA::TrackballManipulator );
   // set up the camera manipulators.
   {
       osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

       keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
       keyswitchManipulator->addMatrixManipulator( '2', "FirstPerson", new MyFirstPersonManipulator() );
       keyswitchManipulator->addMatrixManipulator( '3', "Flight", new osgGA::FlightManipulator() );
       keyswitchManipulator->addMatrixManipulator( '4', "Drive", new osgGA::DriveManipulator() );
       keyswitchManipulator->addMatrixManipulator( '5', "Terrain", new osgGA::TerrainManipulator() );
       keyswitchManipulator->addMatrixManipulator( '6', "Orbit", new osgGA::OrbitManipulator() );
       keyswitchManipulator->addMatrixManipulator( '7', "Spherical", new osgGA::SphericalManipulator() );
       keyswitchManipulator->addMatrixManipulator( '8', "UFO", new osgGA::UFOManipulator() );
       //std::string pathfile;
       //double animationSpeed = 1.0;
       //while(arguments.read("--speed",animationSpeed) ) {}
       //char keyForAnimationPath = '8';
       /*while (arguments.read("-p",pathfile))
       {
           osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
           if (apm || !apm->valid())
           {
               apm->setTimeScale(animationSpeed);
               unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
               keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
               keyswitchManipulator->selectMatrixManipulator(num);
               ++keyForAnimationPath;
           }
       }*/

       view->setCameraManipulator( keyswitchManipulator.get() );

       //keyswitchManipulator->getCurrentMatrixManipulator()->
   }
   view->addEventHandler(new osgViewer::ThreadingHandler);
   view->addEventHandler(new osgViewer::WindowSizeHandler);
   view->addEventHandler(new osgViewer::HelpHandler());
   view->addEventHandler(new osgViewer::ScreenCaptureHandler);
   view->addEventHandler(new osgViewer::LODScaleHandler);
   view->addEventHandler(new osgViewer::RecordCameraPathHandler);
   //view->addEventHandler(new osgViewer::);

   //m_pickHandler = new PickHandler();
   //view->addEventHandler(m_pickHandler);

   // disable escape key
   setKeyEventSetsDone(0);

   view->addEventHandler(new CameraHandler);

   return gw->getGLWidget();
}
////////////////////////////////////////////////////////////////////////////////
osgQt::GraphicsWindowQt* osgQtWidget::createGraphicsWindow(int x, int y, int w, int h, const std::string& name, bool windowDecoration)
{
   osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
   osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
   traits->windowName = name;
   traits->windowDecoration = windowDecoration;
   traits->x = x;
   traits->y = y;
   traits->width = w;
   traits->height = h;
   traits->doubleBuffer = true;
   traits->alpha = ds->getMinimumNumAlphaBits();
   //traits->depth = 24;
   traits->stencil = ds->getMinimumNumStencilBits();
   traits->sampleBuffers = ds->getMultiSamples();
   //traits->samples = ds->getNumMultiSamples();
   traits->samples = 4;

   return new osgQt::GraphicsWindowQt(traits.get());
}
////////////////////////////////////////////////////////////////////////////////
void osgQtWidget::setPickHandler(PickHandler* pickhandler)
{
    m_pickHandler = pickhandler;
    m_osgView->addEventHandler(m_pickHandler);
}
////////////////////////////////////////////////////////////////////////////////
void osgQtWidget::paintEvent(QPaintEvent* /*event*/)
{
   frame();
}
////////////////////////////////////////////////////////////////////////////////
void osgQtWidget::setSceneData(osg::Node* scene)
{
    if(scene)
    {
        m_osgView->setSceneData(scene);
        //m_osgView->addEventHandler(new osgGA::StateSetManipulator(m_osgScene->getOrCreateStateSet()));

        osg::ref_ptr<osgGA::StateSetManipulator> manip = new osgGA::StateSetManipulator(scene->getOrCreateStateSet());
        //manip->set
        m_osgView->addEventHandler(manip);
    }
}
////////////////////////////////////////////////////////////////////////////////
/*const std::string& osgQtWidget::getNodePicked() const
{
    return m_pickHandler->getNodePicked();
}*/
////////////////////////////////////////////////////////////////////////////////
void osgQtWidget::centerCamera()
{
    m_osgView->home();
}
////////////////////////////////////////////////////////////////////////////////
PickHandler* osgQtWidget::getPickHandler()
{
    return m_pickHandler;
}
////////////////////////////////////////////////////////////////////////////////
QWidget* osgQtWidget::getWidget()
{
    return m_widget;
}
////////////////////////////////////////////////////////////////////////////////
void osgQtWidget::setActive(bool val, int freq)
{
    if(val)
    {
        m_timer.setInterval(15);
    }
    else
    {
        m_timer.setInterval(freq);
    }
}
////////////////////////////////////////////////////////////////////////////////
