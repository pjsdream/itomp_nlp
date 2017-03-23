#include <itomp_nlp/renderer/rendering_robot.h>

#include <itomp_nlp/renderer/renderer.h>
#include <itomp_nlp/renderer/rendering_mesh_manager.h>

#include <itomp_nlp/robot/joint.h>


namespace itomp
{

RenderingRobot::RenderingRobot(Renderer* renderer, RobotModel* robot_model)
    : RenderingShape(renderer)
    , robot_model_(robot_model)
    , robot_state_(robot_model)
{
}

void RenderingRobot::draw(ShaderProgram* shader)
{
    shader_ = shader;

    drawLinks(robot_model_->getRootLink(), Eigen::Affine3d::Identity());
}

void RenderingRobot::drawLinks(const Link* link, Eigen::Affine3d transform)
{
    const std::vector<std::string>& visual_mesh_filenames = link->getVisualMeshFilenames();
    const std::vector<Eigen::Affine3d>& visual_mesh_origins = link->getVisualOrigins();

    for (int i=0; i<visual_mesh_filenames.size(); i++)
    {
        // TODO: visual mesh lookup takes O(lg n) but optimal is O(1)
        const std::string& visual_mesh_filename = visual_mesh_filenames[i];
        RenderingMesh* mesh = RenderingMeshManager::getMesh(renderer_, visual_mesh_filename);

        const Eigen::Affine3d& visual_mesh_origin = visual_mesh_origins[i];
        mesh->setTransform(transform * visual_mesh_origin);

        mesh->draw(shader_);
    }

    for (int i=0; i<link->getNumChild(); i++)
    {
        // TODO: joint angle value lookup takes O(lg n) but optimal is O(1)
        const Joint* joint = link->getChildJoint(i);
        const Eigen::Affine3d& joint_origin = joint->getOrigin();
        const Eigen::Affine3d joint_transform = joint->getTransform( robot_state_.getPosition(joint->getJointName()) );

        drawLinks(link->getChildLink(i), transform * joint_origin * joint_transform);
    }
}

void RenderingRobot::setRobotState(const RobotState& robot_state)
{
    robot_state_ = robot_state;
}

}
