////////////////////////////////////////////////////////////////////////////////
#include "gui/moc/mainWindow.hpp"
#include <QApplication>

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{

#ifdef Q_WS_X11
    // FIXME: how come there is no equivalent for QT5 (that renamed
    // the Q_WS_X11 preprocessing symbol to be Q_OS_X11) ?
    QCoreApplication::setAttribute(Qt::AA_X11InitThreads);
#endif

    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
////////////////////////////////////////////////////////////////////////////////
