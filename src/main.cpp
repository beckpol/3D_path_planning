#include "mainwindow.h"
#include <string.h>
#include <QApplication>

"main function"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    if(argc==2 && !strcmp(argv[1],"-v")){w.verbose=1;}
    w.show();
    return a.exec();
}
