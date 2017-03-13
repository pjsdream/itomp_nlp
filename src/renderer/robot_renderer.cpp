#include <itomp_nlp/renderer/robot_renderer.h>

#include <itomp_nlp/robot/joint.h>


namespace itomp
{

RobotRenderer::RobotRenderer(Renderer* renderer, RobotModel* robot_model)
    : renderer_(renderer)
{
    renderer->makeCurrent();

    addRobotObjects(robot_model);

    renderer->doneCurrent();
}

int RobotRenderer::addRobotEntity()
{
    std::vector<int> entities;

    addRobotEntitiesRecursive(robot_model_->getRootLink(), Eigen::Affine3d::Identity(), entities);

    robot_entities_.push_back(entities);
    return robot_entities_.size() - 1;
}

void RobotRenderer::setRobotEntity(int entity_id, RobotState* robot_state)
{
    int id = 0;
    setRobotEntitiesRecursive(robot_model_->getRootLink(), Eigen::Affine3d::Identity(), robot_entities_[entity_id], id, robot_state);
}

void RobotRenderer::addRobotObjects(RobotModel* robot_model)
{
    robot_model_ = robot_model;
    robot_entities_.push_back(std::vector<int>());

    addRobotObjectsRecursive(robot_model->getRootLink());
}

void RobotRenderer::addRobotObjectsRecursive(const Link* link)
{
    const std::vector<std::string>& mesh_filenames = link->getVisualMeshFilenames();

    for (int i=0; i<mesh_filenames.size(); i++)
    {
        const int object = renderer_->registerMeshFile(mesh_filenames[i]);
        robot_objects_.push_back(object);
    }

    for (int i=0; i<link->getNumChild(); i++)
        addRobotObjectsRecursive( link->getChildLink(i) );
}

void RobotRenderer::addRobotEntitiesRecursive(const Link* link, const Eigen::Affine3d transform, std::vector<int>& entities)
{
    // WARNING! it should recurses in the same manner objects are registered
    
    const std::vector<Eigen::Affine3d>& mesh_origins = link->getVisualOrigins();

    for (int i=0; i<mesh_origins.size(); i++)
    {
        const int object_id = robot_objects_[ entities.size() ];
        const int entity_id = renderer_->addEntity(object_id, transform * mesh_origins[i]);
        entities.push_back(entity_id);
    }

    for (int i=0; i<link->getNumChild(); i++)
    {
        Joint* joint = link->getChildJoint(i);

        const Eigen::Affine3d& joint_origin = joint->getOrigin();

        addRobotEntitiesRecursive( link->getChildLink(i), transform * joint_origin, entities );
    }
}

void RobotRenderer::setRobotEntitiesRecursive(const Link* link, const Eigen::Affine3d transform, const std::vector<int>& entities, int& entity_idx, RobotState* robot_state)
{
    // WARNING! it should recurses in the same manner objects are registered
    
    const std::vector<Eigen::Affine3d>& mesh_origins = link->getVisualOrigins();

    for (int i=0; i<mesh_origins.size(); i++)
    {
        renderer_->setEntityTransform(entities[entity_idx++], transform * mesh_origins[i]);
    }

    for (int i=0; i<link->getNumChild(); i++)
    {
        Joint* joint = link->getChildJoint(i);

        const Eigen::Affine3d& joint_transform = joint->getOrigin() * joint->getTransform( robot_state->getPosition(joint->getJointName()) );

        setRobotEntitiesRecursive( link->getChildLink(i), transform * joint_transform, entities, entity_idx, robot_state );
    }
    
}

}
