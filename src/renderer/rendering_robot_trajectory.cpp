#include <itomp_nlp/renderer/rendering_robot_trajectory.h>


namespace itomp
{

RenderingRobotTrajectory::RenderingRobotTrajectory(Renderer* renderer, RobotModel* robot_model, const RobotState& robot_state)
    : RenderingShape(renderer)
    , robot_model_(robot_model)
    , robot_state_(robot_state)
    , rendering_robot_(renderer, robot_model)
    , rendering_object_(0)
{
}

void RenderingRobotTrajectory::draw(ShaderProgram* shader)
{
    const Eigen::MatrixXd m = trajectory_.getTrajectory();
    if (m.rows() == 0 || m.cols() == 0)
        return;

    const double t = time_ / trajectory_.getDuration();
    const int num_waypoints = m.cols() / 2;

    int i = t * (num_waypoints - 1);
    double u;
    if (i >= num_waypoints - 1)
    {
        i = num_waypoints - 2;
        u = 1.;
    }
    else
        u = t * (num_waypoints - 1) - i;

    Eigen::VectorXd p0 = m.col(2*i);
    Eigen::VectorXd v0 = m.col(2*i+1);
    Eigen::VectorXd p1 = m.col(2*i+2);
    Eigen::VectorXd v1 = m.col(2*i+3);

    const double u2 = u*u;
    const double u3 = u2*u;

    Eigen::VectorXd positions = 
        (2*u3 - 3*u2 + 1) * p0
        + (u3 - 2*u2 + u) * v0
        + (-2*u3 + 3*u2) * p1
        + (u3 - u2) * v1;

    const std::vector<std::string> joint_names = trajectory_.getJointNames();
    for (int i=0; i<joint_names.size(); i++)
        robot_state_.setPosition(joint_names[i], positions[i]);

    rendering_robot_.setRobotState(robot_state_);

    rendering_robot_.draw(shader);

    // update endeffector transform
    if (rendering_object_ != 0)
    {
        forward_kinematics_->setPositions(positions);
        forward_kinematics_->setVelocities(positions);
        forward_kinematics_->forwardKinematics();

        Eigen::Affine3d transform = forward_kinematics_->getLinkWorldTransform(7);
        transform.translate(Eigen::Vector3d(0.2, 0, 0));
        rendering_object_->setTransform(transform);
    }
}

void RenderingRobotTrajectory::setForwardKinematics(OptimizerRobot* forward_kinematics)
{
    forward_kinematics_ = forward_kinematics;
}

void RenderingRobotTrajectory::attachObject(RenderingShape* object)
{
    rendering_object_ = object;
}

void RenderingRobotTrajectory::setRobotTrajectory(const Trajectory& trajectory)
{
    trajectory_ = trajectory;
}

void RenderingRobotTrajectory::setTime(double time)
{
    time_ = time;
}

}
