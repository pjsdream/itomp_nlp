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

    for (int i=0; i<fk_shapes_.size(); i++)
    {
        for (int j=0; j<fk_shapes_[i].size(); j++)
            delete fk_shapes_[i][j];
    }
}

OptimizerRobot::OptimizerRobot(const OptimizerRobot& robot)
{
    links_ = robot.links_;
    joints_ = robot.joints_;

    positions_ = robot.positions_;
    velocities_ = robot.velocities_;

    link_world_transforms_ = robot.link_world_transforms_;

    fk_shapes_ = robot.fk_shapes_;

    // create new shapes
    for (int i=0; i<robot.links_.size(); i++)
    {
        const std::vector<itomp_shape::Shape*>& shapes = robot.links_[i].shapes;
        for (int j=0; j<shapes.size(); j++)
        {
            const itomp_shape::Shape* shape = shapes[j];
            links_[i].shapes[j] = shape->clone();
        }
    }

    for (int i=0; i<fk_shapes_.size(); i++)
    {
        for (int j=0; j<robot.fk_shapes_[i].size(); j++)
            fk_shapes_[i][j] = robot.fk_shapes_[i][j]->clone();
    }
}

void OptimizerRobot::setLinkJoints(const std::vector<Link>& links, const std::vector<Joint>& joints)
{
    links_ = links;
    joints_ = joints;

    // resize cache
    link_world_transforms_.resize(links.size());

    // forward kinematics shapes
    fk_shapes_.resize(links_.size());
    for (int i=0; i<links_.size(); i++)
    {
        const std::vector<itomp_shape::Shape*>& shapes = links_[i].shapes;
        for (int j=0; j<shapes.size(); j++)
        {
            fk_shapes_[i].push_back( shapes[j]->clone() );
        }
    }
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
    for (int i=0; i<links_.size(); i++)
    {
        const Link& link = links_[i];
        for (int j=0; j<link.shapes.size(); j++)
        {
            const itomp_shape::Shape* shape = link.shapes[j];

            // TODO: code-level optimization of eigen matrix multiplication
            fk_shapes_[i][j]->setTransform( link_world_transforms_[i] * shape->getTransform() );
        }
    }
}

}
