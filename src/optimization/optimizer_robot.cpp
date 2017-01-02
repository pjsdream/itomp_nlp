#include <itomp_nlp/optimization/optimizer_robot.h>

#include <itomp_nlp/robot/revolute_joint.h>
#include <itomp_nlp/robot/prismatic_joint.h>
#include <itomp_nlp/robot/fixed_joint.h>


namespace itomp_optimization
{

const double OptimizerRobot::position_lower_default_ = -10.;
const double OptimizerRobot::position_upper_default_ = 10.;

const double OptimizerRobot::velocity_lower_default_ = -100.;
const double OptimizerRobot::velocity_upper_default_ = 100.;


OptimizerRobot::OptimizerRobot()
{
}

OptimizerRobot::~OptimizerRobot()
{
    // delete cloned collision shapes
    for (int i=0; i<links_.size(); i++)
    {
        const Link& link = links_[i];
        for (int j=0; j<link.shapes.size(); j++)
            delete link.shapes[j];
    }
}

void OptimizerRobot::setLinkJoints(const std::vector<Link>& links, const std::vector<Joint>& joints)
{
    links_ = links;
    joints_ = joints;
}

void OptimizerRobot::setPositions(const Eigen::VectorXd& positions)
{
    // TODO: clamping
    positions_ = positions;
}

void OptimizerRobot::setVelocities(const Eigen::VectorXd& velocities)
{
    // TODO: clamping
    velocities_ = velocities;
}

void OptimizerRobot::setBaseTransform(const Eigen::Affine3d& transform)
{
    link_world_transforms_[0] = transform;
}

void OptimizerRobot::forwardKinematics()
{
    for (int i=1; i<links_.size(); i++)
    {
        const Joint& joint = joints_[i];

        link_world_transforms_[i] = link_world_transforms_[joint.parent] * joint.origin;

        // joint transform
        switch (joint.joint_type)
        {
        case JOINT_TYPE_PRISMATIC:
            link_world_transforms_[i].translate(joint.axis * positions_[i-1]);
            break;

        case JOINT_TYPE_REVOLUTE:
            link_world_transforms_[i].rotate( Eigen::AngleAxisd(positions_[i-1], joint.axis) );
            break;
        }
    }

    // collision shape update
    int idx = 0;
    for (int i=0; i<links_.size(); i++)
    {
        const Link& link = links_[i];
        for (int j=0; j<link.shapes.size(); j++)
        {
            const itomp_shape::Shape* shape = link.shapes[j];

            // TODO: code-level optimization of eigen matrix multiplication
            fk_shapes_[idx]->setTransform( link_world_transforms_[i] * shape->getTransform() );
        }
    }
}

}
