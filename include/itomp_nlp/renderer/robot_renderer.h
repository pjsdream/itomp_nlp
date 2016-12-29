#ifndef ITOMP_RENDERER_ROBOT_RENDERER_H
#define ITOMP_RENDERER_ROBOT_RENDERER_H


#include <itomp_nlp/robot/robot_model.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp_renderer
{

class RobotRenderer
{
public:

    RobotRenderer(Renderer* renderer, itomp_robot::RobotModel* robot_model);

    int addRobotEntity();

private:

    void addRobotObjects(itomp_robot::RobotModel* robot_model);
    void addRobotObjectsRecursive(const itomp_robot::Link* link);
    void addRobotEntitiesRecursive(const itomp_robot::Link* link, const Eigen::Affine3d transform, std::vector<int>& entities);

    Renderer* renderer_;

    // robot model and corresponding meshes
    itomp_robot::RobotModel* robot_model_;
    std::vector<int> robot_objects_;

    // entities
    std::vector<std::vector<int> > robot_entities_;
};

}


#endif // ITOMP_RENDERER_ROBOT_RENDERER_H