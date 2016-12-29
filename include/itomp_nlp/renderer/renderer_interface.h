#ifndef ITOMP_RENDERER_SIMULATOR_INTERFACE_H
#define ITOMP_RENDERER_SIMULATOR_INTERFACE_H


#include <cmath>

#include <QMainWindow>

#include <itomp_nlp/renderer/renderer.h>
#include <itomp_nlp/renderer/robot_renderer.h>


namespace itomp_renderer
{

class RendererInterface : public QMainWindow
{
    Q_OBJECT

public:

    RendererInterface();

    void addRobot(itomp_robot::RobotModel* robot_model);
    void addRobotEntity(int robot_index);

protected slots:

    void updateNextFrame();

private:

    Renderer* renderer_;

    std::vector<RobotRenderer*> robot_renderers_;
};

}


#endif // ITOMP_RENDERER_SIMULATOR_INTERFACE_H
