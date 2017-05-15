#include <itomp_nlp/network/trajectory_subscriber.h>
#include <itomp_nlp/ros/fetch.h>

#include <stdio.h>

#include <ros/ros.h>


int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    ros::init(argc, argv, "trajectory_executor");

    itomp::Fetch fetch;

    // initialize with highest torso position
    ROS_INFO("initializing torso position to 0.35m");
    fetch.moveTorso(0.35);

    // open gripper at start
    ROS_INFO("opening gripper and wait for 3 seconds");
    fetch.openGripper();
    ros::Duration(3.0).sleep();

    // picking a can
    ROS_INFO("picking a can");
    fetch.moveGripper(0.06, true);

    ROS_INFO("initializing arm position to zero");
    fetch.moveArmZeroPosition();

    //test_fetch.runMovingArmScenario();
    //test_fetch.runScenario();

    setbuf(stdout, NULL);

    itomp::TrajectorySubscriber subscriber("localhost");

    while (ros::ok())
    {
        itomp::Trajectory trajectory;
        trajectory = subscriber.receiveSync();

        ROS_INFO("time: %lf", ros::Time::now().toSec());
        fetch.moveArm(ros::Time::now(), trajectory);
    }

    return 0;
}
