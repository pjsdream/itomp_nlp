#include <QApplication>
#include <cstdlib>
#include <stdio.h>

#include <itomp_nlp/renderer/renderer_interface.h>

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    QApplication app(argc, argv);
    itomp_renderer::RendererInterface* renderer_interface = new itomp_renderer::RendererInterface();

    renderer_interface->show();

    app.exec();

    return 0;
}
