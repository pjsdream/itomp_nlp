#include <QApplication>
#include <cstdlib>
#include <stdio.h>

#include <itomp_nlp/interface/main_window.h>


int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    QApplication app(argc, argv);
    itomp::MainWindow* main_window = new itomp::MainWindow();
    main_window->show();

    app.exec();

    return 0;
}
