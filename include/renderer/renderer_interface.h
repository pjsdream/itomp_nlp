#ifndef ITOMP_RENDERER_SIMULATOR_INTERFACE_H
#define ITOMP_RENDERER_SIMULATOR_INTERFACE_H


#include <cmath>

#include <QMainWindow>

#include <renderer/renderer.h>


namespace itomp_renderer
{

class RendererInterface : public QMainWindow
{
    Q_OBJECT

public:

    RendererInterface();

protected slots:

    void updateNextFrame();

private:

    Renderer* renderer_;
};

}


#endif // ITOMP_RENDERER_SIMULATOR_INTERFACE_H
