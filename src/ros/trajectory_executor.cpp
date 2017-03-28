#include <itomp_nlp/network/trajectory_subscriber.h>
#include <itomp_nlp/ros/fetch.h>

#include <stdio.h>

#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_executor");

    itomp::Fetch fetch;

    // initialize with highest torso position
    fetch.moveTorso(0.35);

    // open/close gripper at start
    fetch.moveGripper(0.01);
    fetch.openGripper();

    fetch.moveArmZeroPosition();

    //test_fetch.runMovingArmScenario();
    //test_fetch.runScenario();

    ros::Rate rate(2);

    setbuf(stdout, NULL);

    itomp::TrajectorySubscriber subscriber("152.23.47.51");

    ros::Time stamp = ros::Time::now();
    while (ros::ok())
    {
        ROS_INFO("supposed time: %lf", stamp.toSec());

        itomp::Trajectory trajectory;
        trajectory = subscriber.receive();

        fetch.moveArm(stamp, trajectory);

        rate.sleep();
        stamp += ros::Duration(0.5);
    }

    return 0;
}
