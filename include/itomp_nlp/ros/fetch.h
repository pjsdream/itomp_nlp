/*
 * $ roslaunch itomp_exec itomp_fetch.launch use_gazebo:=true benchmark1:=true
 * $ rosparam load itomp_fetch_benchmark1.yaml itomp_fetch
 */

#ifndef ITOMP_NLP_ROS_FETCH_H
#define ITOMP_NLP_ROS_FETCH_H


#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <itomp_nlp/optimization/trajectory.h>


namespace itomp
{

class Fetch
{
private:
    
    static const std::vector<std::string> joint_names_;

public:
    
    Fetch(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    void moveGripper(double gap, bool wait_for_execution = true);
    void openGripper(bool wait_for_execution = true);
    void closeGripper(bool wait_for_execution = true);
    
    void moveTorso(double position, bool wait_for_execution = true);

    void moveArmZeroPosition();
    void moveArm(const ros::Time start_time, const Trajectory& trajectory);

private:

    ros::NodeHandle nh_;
    ros::Publisher start_state_publisher_;
    ros::Publisher goal_state_publisher_;
    ros::Publisher display_trajectory_publisher_;
    ros::Publisher target_positions_publisher_;
    
    // gripper actionlib client
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client_;
    
    // torso actionlib client
    // rostopic pub -1 /torso_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal '{header:{}, goal_id:{}, goal:{trajectory : {joint_names: [torso_lift_joint], points: [{positions : [0.35], time_from_start: [5.0, 0.0]}]}}}'
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client_;
};

}


#endif // ITOMP_NLP_ROS_FETCH_H
