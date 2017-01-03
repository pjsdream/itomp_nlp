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

    itomp_robot::URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "/home/jaesungp/catkin_ws/src/fetch_ros/fetch_description");
    itomp_robot::RobotModel* robot_model = urdf_parser.parseURDF("/home/jaesungp/catkin_ws/src/itomp_nlp/urdf/fetch.urdf");

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

    itomp_optimization::Optimizer optimizer;
    optimizer.setRobot(optimizer_robot);

    renderer_interface->addRobot(robot_model);
    renderer_interface->addRobotEntity(0);

    renderer_interface->show();

    app.exec();

    return 0;
}
