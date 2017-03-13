#ifndef ITOMP_RENDERER_ROBOT_RENDERER_H
#define ITOMP_RENDERER_ROBOT_RENDERER_H


#include <itomp_nlp/robot/robot_model.h>
#include <itomp_nlp/robot/robot_state.h>

#include <itomp_nlp/renderer/renderer.h>


namespace itomp
{

class RobotRenderer
{
public:

    RobotRenderer(Renderer* renderer, RobotModel* robot_model);

    int addRobotEntity();
    void setRobotEntity(int entity_id, RobotState* robot_state);

private:

    void addRobotObjects(RobotModel* robot_model);
    void addRobotObjectsRecursive(const Link* link);
    void addRobotEntitiesRecursive(const Link* link, const Eigen::Affine3d transform, std::vector<int>& entities);

    void setRobotEntitiesRecursive(const Link* link, const Eigen::Affine3d transform, const std::vector<int>& entities, int& entity_idx, RobotState* robot_state);

    Renderer* renderer_;

    // robot model and corresponding meshes
    RobotModel* robot_model_;
    std::vector<int> robot_objects_;

    // entities
    std::vector<std::vector<int> > robot_entities_;
};

}


#endif // ITOMP_RENDERER_ROBOT_RENDERER_H