#include "headpose.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    headpose w;
    w.show();

    return a.exec();
}
