#include <itomp_nlp/optimization/optimizer_robot.h>

#include <itomp_nlp/robot/revolute_joint.h>
#include <itomp_nlp/robot/prismatic_joint.h>
#include <itomp_nlp/robot/fixed_joint.h>

#include <iostream>


namespace itomp
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
    link_world_transform_derivatives_ = robot.link_world_transform_derivatives_;

    fk_shapes_ = robot.fk_shapes_;

    // create new shapes
    for (int i=0; i<robot.links_.size(); i++)
    {
        const std::vector<Shape*>& shapes = robot.links_[i].shapes;
        for (int j=0; j<shapes.size(); j++)
        {
            const Shape* shape = shapes[j];
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
    link_world_transform_derivatives_.resize(links.size());
    setBaseTransform(Eigen::Affine3d::Identity());

    // forward kinematics shapes
    fk_shapes_.resize(links_.size());
    for (int i=0; i<links_.size(); i++)
    {
        const std::vector<Shape*>& shapes = links_[i].shapes;
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
    link_world_transform_derivatives_[0] = transform.matrix();
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
            const Shape* shape = link.shapes[j];

            // TODO: code-level optimization of eigen matrix multiplication
            fk_shapes_[i][j]->setTransform( link_world_transforms_[i] * shape->getTransform() );
        }
    }

    // velocity forward kinematics
    for (int i=1; i<links_.size(); i++)
    {
        const Joint& joint = joints_[i];
        
        // joint transform including origin
        Eigen::Matrix4d joint_transform = joint.origin.matrix();
        Eigen::Matrix4d joint_transform_derivative = joint.origin.matrix();

        switch (joint.joint_type)
        {
        case JOINT_TYPE_PRISMATIC:
            joint_transform *= Eigen::Affine3d( Eigen::Translation3d(joint.axis * positions_[i-1]) ).matrix();
            joint_transform_derivative *= prismaticJointDerivative(i);
            break;

        case JOINT_TYPE_REVOLUTE:
            joint_transform *= Eigen::Affine3d( Eigen::AngleAxisd(positions_[i-1], joint.axis) ).matrix();
            joint_transform_derivative *= revoluteJointDerivative(i);
            break;
        }

        link_world_transform_derivatives_[i] = 
            link_world_transform_derivatives_[joint.parent] * joint_transform
            + link_world_transforms_[joint.parent].matrix() * joint_transform_derivative;
    }
}

Eigen::Matrix4d OptimizerRobot::prismaticJointDerivative(int joint_idx)
{
    const Joint& joint = joints_[joint_idx];
    const Eigen::Vector3d& axis = joint.axis;
    const double& ax = axis(0);
    const double& ay = axis(1);
    const double& az = axis(2);

    const double& position = positions_[joint_idx-1];
    const double& velocity = velocities_[joint_idx-1];

    Eigen::Matrix4d m;
    m.setZero();

    m(0, 3) = ax * velocity;
    m(1, 3) = ay * velocity;
    m(2, 3) = az * velocity;

    return m;
}

Eigen::Matrix4d OptimizerRobot::revoluteJointDerivative(int joint_idx)
{
    const Joint& joint = joints_[joint_idx];
    const Eigen::Vector3d& axis = joint.axis;
    const double& ax = axis(0);
    const double& ay = axis(1);
    const double& az = axis(2);

    const double& position = positions_[joint_idx-1];
    const double& velocity = velocities_[joint_idx-1];
    const double c = std::cos(position);
    const double s = std::sin(position);

    const Eigen::Vector4d q (c, ax * s, ay * s, az * s);
    const Eigen::Vector4d dq = velocity * Eigen::Vector4d(-s, ax * c, ay * c, az * c);

    Eigen::Matrix4d m;
    m.setZero();

    m(0, 0) = - 4. * q(2) * dq(2) - 4. * q(3) * dq(3);
    m(1, 0) = 2. * dq(1) * q(2) + 2. * q(1) * dq(2) + 2. * dq(3) * q(0) + 2. * q(3) * dq(0);
    m(2, 0) = 2. * dq(1) * q(3) + 2. * q(1) * dq(3) - 2. * dq(2) * q(0) - 2. * q(2) * dq(0);

    m(0, 1) = 2. * dq(1) * q(2) + 2. * q(1) * dq(2) - 2. * dq(3) * q(0) - 2. * q(3) * dq(0);
    m(1, 1) = - 4. * q(1) * dq(1) - 4. * q(3) * dq(3);
    m(2, 1) = 2. * dq(2) * q(3) + 2. * q(2) * dq(3) + 2. * dq(1) * q(0) + 2. * q(1) * dq(0);

    m(0, 2) = 2. * dq(1) * q(3) + 2. * q(1) * dq(3) + 2. * dq(2) * q(0) + 2. * q(2) * dq(0);
    m(1, 2) = 2. * dq(2) * q(3) + 2. * q(2) * dq(3) - 2. * dq(1) * q(0) - 2. * q(1) * dq(0);
    m(2, 2) = - 4. * q(1) * dq(1) - 4. * q(2) * dq(2);

    return m;
}

Eigen::Vector3d OptimizerRobot::getWorldVelocity(int link_idx, const Eigen::Vector3d& local_position)
{
    const Eigen::Matrix4d& m = link_world_transform_derivatives_[link_idx];

    Eigen::Vector4d v;
    v << local_position, 0.;

    return (m * v).block(0, 0, 3, 1);
}

}
