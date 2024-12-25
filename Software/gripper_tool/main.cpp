#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyleSheet("QWidget { font-size: 18px; }");
    a.addLibraryPath("./");
    MainWindow w;

    w.show();

    return a.exec();
}
