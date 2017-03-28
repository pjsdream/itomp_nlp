#include <itomp_nlp/ros/fetch.h>


#include <moveit_msgs/RobotState.h>
#include <eigen_conversions/eigen_msg.h>
#include <control_msgs/GripperCommandGoal.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <stdlib.h>
#include <time.h>


namespace itomp
{

const std::vector<std::string> Fetch::joint_names_ =
{
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "upperarm_roll_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
};


Fetch::Fetch(const ros::NodeHandle& nh)
    : nh_(nh)
    , gripper_client_("gripper_controller/gripper_action")
    , torso_client_("torso_controller/follow_joint_trajectory")
    , arm_client_("arm_controller/follow_joint_trajectory")
{
    ROS_INFO("waiting for arm controller server");
    arm_client_.waitForServer();
}

void Fetch::moveGripper(double gap, bool wait_for_execution)
{
    control_msgs::GripperCommandGoal goal;
    goal.command.position = gap;
    goal.command.max_effort = 1000.;
    gripper_client_.sendGoal(goal);
    
    if (wait_for_execution)
        gripper_client_.waitForResult();
}

void Fetch::closeGripper(bool wait_for_execution)
{
    moveGripper(0.00, wait_for_execution);
}

void Fetch::openGripper(bool wait_for_execution)
{
    moveGripper(0.10, wait_for_execution);
}

void Fetch::moveTorso(double position, bool wait_for_execution)
{
    const double time = 1.0;
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(position);
    point.time_from_start = ros::Duration(time);
    
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    goal.trajectory.points.push_back(point);
    
    torso_client_.sendGoal(goal);
    
    if (wait_for_execution)
    {
        if (!torso_client_.waitForResult())
            ROS_ERROR("Torso action client failed to finish trajectory execution");
    }
}

void Fetch::moveArmZeroPosition()
{
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint point;

    point.time_from_start = ros::Duration(3.0);
    for (int i=0; i<joint_names_.size(); i++)
    {
        goal.trajectory.joint_names.push_back(joint_names_[i]);
        point.positions.push_back(0);
    }

    goal.trajectory.points.push_back(point);

    arm_client_.sendGoal(goal);
    arm_client_.waitForResult();
}

void Fetch::moveArm(const ros::Time start_time, const Trajectory& trajectory)
{
    const std::vector<std::string> joint_names = trajectory.getJointNames();
    const double duration = trajectory.getDuration();
    const Eigen::MatrixXd& m = trajectory.getTrajectory();

    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(m.rows());

    goal.trajectory.joint_names = joint_names;

    ros::Time current_time = ros::Time::now();
    goal.trajectory.header.stamp = current_time;

    const int num_waypoints = m.cols() / 2;
    for (int i=0; i<num_waypoints; i++)
    {
        const ros::Duration t((double)(i+1) / num_waypoints * duration);
        if (current_time < start_time + t)
        {
            point.time_from_start = start_time + t - current_time;
            for (int j=0; j<m.rows(); j++)
                point.positions[j] = m(j, i*2);

            goal.trajectory.points.push_back(point);
        }
        else
        {
            printf("waypoint ignored\n");
        }
    }

    arm_client_.sendGoal(goal);
}

}
