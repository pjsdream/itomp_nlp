#include <QApplication>
#include <cstdlib>
#include <stdio.h>

#include <itomp_nlp/renderer/renderer_interface.h>

#include <itomp_nlp/robot/urdf_parser.h>

#include <itomp_nlp/optimization/optimizer.h>
#include <itomp_nlp/optimization/optimizer_robot_loader.h>


int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    QApplication app(argc, argv);
    itomp_renderer::RendererInterface* renderer_interface = new itomp_renderer::RendererInterface();

#ifdef WIN32
    itomp_robot::URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "..");
    itomp_robot::RobotModel* robot_model = urdf_parser.parseURDF("../urdf/fetch.urdf");
#else
    itomp_robot::URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "/home/jaesungp/catkin_ws/src/fetch_ros/fetch_description");
    itomp_robot::RobotModel* robot_model = urdf_parser.parseURDF("/home/jaesungp/catkin_ws/src/itomp_nlp/urdf/fetch.urdf");
#endif

    // default robot state
    itomp_robot::RobotState* robot_state = new itomp_robot::RobotState(robot_model);

    std::vector<std::string> active_joint_names = 
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    };

    std::vector<std::vector<std::string> > aabb_lists = 
    {
        {
            "base_link",
        },
        {
            "torso_lift_link",
            "torso_fixed_link",
        },
        {
            "head_pan_link",
            "head_tilt_link",
        },
        {
            "shoulder_pan_link",
        },
        {
            "shoulder_lift_link",
        },
        {
            "upperarm_roll_link",
        },
        {
            "elbow_flex_link",
        },
        {
            "forearm_roll_link",
        },
        {
            "wrist_flex_link",
        },
        {
            "wrist_roll_link",
            "gripper_link",
        },
        {
            "r_gripper_finger_link",
        },
        {
            "l_gripper_finger_link",
        },
    };

    itomp_optimization::OptimizerRobotLoader optimizer_robot_loader;

    for (int i=0; i<aabb_lists.size(); i++)
        optimizer_robot_loader.addAABBList(aabb_lists[i]);

    itomp_optimization::OptimizerRobot* optimizer_robot = optimizer_robot_loader.loadRobot(robot_model, robot_state, active_joint_names);

    itomp_optimization::OptimizerOptions options;
    options.trajectory_duration = 3.0;
    options.timestep = 0.5;
    options.num_waypoints = 6;
    options.num_waypoint_interpolations = 8;

    itomp_optimization::Optimizer optimizer;
    optimizer.setRobot(optimizer_robot);
    optimizer.setOptions(options);
    optimizer.prepare();

    Eigen::Matrix<double, 7, 1> position;
    Eigen::Matrix<double, 7, 1> velocity;
    position.setZero();
    velocity.setZero();
    optimizer.setInitialRobotState(position, velocity);

    // end effector link id = 7
    optimizer.setGoalPosition(7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(1, 1, 1));
    optimizer.setGoalVelocity(7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0, 0, -1));

    optimizer.startOptimizationThread();

    renderer_interface->addRobot(robot_model);
    renderer_interface->addRobotEntity(0);

    renderer_interface->show();

    app.exec();

    return 0;
}
