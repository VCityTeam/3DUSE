////////////////////////////////////////////////////////////////////////////////
#include "gui/moc/mainWindow.hpp"
#include <QApplication>

#include <geos/geom/Geometry.h>
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	std::cout << "GEOS " << geos::geom::geosversion() << " ported from JTS " << geos::geom::jtsport() << std::endl;

#ifdef Q_WS_X11
    QCoreApplication::setAttribute(Qt::AA_X11InitThreads);
#endif

    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
////////////////////////////////////////////////////////////////////////////////
