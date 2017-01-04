#ifndef ITOMP_RENDERER_SIMULATOR_INTERFACE_H
#define ITOMP_RENDERER_SIMULATOR_INTERFACE_H


#include <cmath>

#include <QMainWindow>

#include <itomp_nlp/renderer/renderer.h>
#include <itomp_nlp/renderer/robot_renderer.h>

#include <itomp_nlp/optimization/optimizer.h>
#include <itomp_nlp/optimization/optimizer_robot_loader.h>


namespace itomp_renderer
{

class RendererInterface : public QMainWindow
{
    Q_OBJECT

public:

    RendererInterface();

    void addRobot(itomp_robot::RobotModel* robot_model);
    void addRobotEntity(int robot_index);
    void setRobotEntity(int robot_index, int entity_id, itomp_robot::RobotState* robot_state);

protected slots:

    void updateNextFrame();

private:

    // resources
    void initializeResources();
    itomp_robot::RobotModel* robot_model_;
    itomp_robot::RobotState* robot_state_;
    itomp_optimization::OptimizerRobot* optimizer_robot_;
    itomp_optimization::Optimizer optimizer_;
    std::vector<std::string> active_joint_names_;
    std::vector<std::vector<std::string> > aabb_lists_;

    Renderer* renderer_;

    std::vector<RobotRenderer*> robot_renderers_;
    std::vector<int> robot_entities_;
};

}


#endif // ITOMP_RENDERER_SIMULATOR_INTERFACE_H
