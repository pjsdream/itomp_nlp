#include <itomp_nlp/renderer/robot_renderer.h>


namespace itomp_renderer
{

RobotRenderer::RobotRenderer(Renderer* renderer, itomp_robot::RobotModel* robot_model)
    : renderer_(renderer)
{
    renderer->makeCurrent();

    addRobotObjects(robot_model);

    renderer->doneCurrent();
}

int RobotRenderer::addRobotEntity()
{
    std::vector<int> entities;

    // TODO: for now, robot entities at origin
    for (int i=0; i<robot_objects_.size(); i++)
        renderer_->addEntity(robot_objects_[i], Eigen::Affine3d(Eigen::Translation3d(0, i * 0.1, 0)));

    robot_entities_.push_back(entities);
    return robot_entities_.size() - 1;
}

void RobotRenderer::addRobotObjects(itomp_robot::RobotModel* robot_model)
{
    robot_models_.push_back(robot_model);
    robot_entities_.push_back(std::vector<int>());

    addRobotObjectsRecursive(robot_model->getRootLink());
}

void RobotRenderer::addRobotObjectsRecursive(const itomp_robot::Link* link)
{
    const std::vector<std::string>& mesh_filenames = link->getVisualMeshFilenames();

    for (int i=0; i<mesh_filenames.size(); i++)
    {
        int object = renderer_->registerMeshFile(mesh_filenames[i]);
        robot_objects_.push_back(object);
    }

    for (int i=0; i<link->getNumChild(); i++)
    {
        addRobotObjectsRecursive( link->getChildLink(i) );
    }
}

}
