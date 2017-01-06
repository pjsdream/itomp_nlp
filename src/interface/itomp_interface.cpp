#include <itomp_nlp/interface/itomp_interface.h>

#include <itomp_nlp/robot/urdf_parser.h>


namespace itomp_interface
{

ItompInterface::ItompInterface(QWidget* parent)
    : QWidget(parent)
{
    resize(400, 600);
    show();

    layout_ = new QGridLayout(this);
    setLayout(layout_);

    start_button_ = new QPushButton("Start", this);
    connect(start_button_, SIGNAL(clicked()), this, SLOT(startMotionPlanning()));

    stop_button_ = new QPushButton("Stop", this);
    connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopMotionPlanning()));

    layout_->addWidget(start_button_, 0, 0);
    layout_->addWidget(stop_button_, 0, 1);

    scroll_area_ = new QScrollArea(this);

    layout_->addWidget(scroll_area_, 1, 0, 1, 2);
    
    // execution tmier
    execution_timer_ = new QTimer(this);
    execution_timer_->setInterval(500);
    connect(execution_timer_, SIGNAL(timeout()), this, SLOT(moveTrajectoryForwardOneTimestep()));
    
    // initialize resource
    initializeResources();

    is_optimizing_ = false;
}

void ItompInterface::initializeResources()
{
#ifdef _WIN32
    itomp_robot::URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "C:\\Users\\jaesungp\\Desktop\\documents\\fetch_ros\\fetch_description");
    robot_model_ = urdf_parser.parseURDF("../urdf/fetch.urdf");
#else
    itomp_robot::URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "/home/jaesungp/catkin_ws/src/fetch_ros/fetch_description");
    itomp_robot::RobotModel* robot_model = urdf_parser.parseURDF("/home/jaesungp/catkin_ws/src/itomp_nlp/urdf/fetch.urdf");
#endif

    // default robot state
    robot_state_ = new itomp_robot::RobotState(robot_model_);
    robot_state_->setPosition("torso_lift_joint", 0.35);

    active_joint_names_ = 
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    };

    aabb_lists_ = 
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

    for (int i=0; i<aabb_lists_.size(); i++)
        optimizer_robot_loader.addAABBList(aabb_lists_[i]);

    optimizer_robot_ = optimizer_robot_loader.loadRobot(robot_model_, robot_state_, active_joint_names_);

    itomp_optimization::OptimizerOptions options;
    options.trajectory_duration = 3.0;
    options.timestep = 0.5;
    options.num_waypoints = 6;
    options.num_waypoint_interpolations = 8;

    optimizer_.setRobot(optimizer_robot_);
    optimizer_.setOptions(options);
    optimizer_.prepare();

    Eigen::Matrix<double, 7, 1> position;
    Eigen::Matrix<double, 7, 1> velocity;
    position.setZero();
    velocity.setZero();
    optimizer_.setInitialRobotState(position, velocity);

    // end effector link id = 7
    optimizer_.setGoalPosition(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0.5, 0.5, 1));
    //optimizer_.setGoalVelocity(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0.5, 0.5, 1), Eigen::Vector3d(0, 0, -0.2));
    /*
    optimizer_.addGoalRegionPlane(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector4d(0, 0,  1, -0.7));
    optimizer_.addGoalRegionPlane(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector4d(0, 0, -1,  0.72));
    optimizer_.addGoalRegionPlane(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector4d( 1, 0, 0, -0.5));
    optimizer_.addGoalRegionPlane(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector4d(-1, 0, 0,  1.0));
    optimizer_.addGoalRegionPlane(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector4d(0,  1, 0,  0.5));
    optimizer_.addGoalRegionPlane(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector4d(0, -1, 0,  0.5));
    optimizer_.addRepulsion(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0.95, 0.01, 0.71), 0.2);
    */
}

Eigen::MatrixXd ItompInterface::getBestTrajectory()
{
    return optimizer_.getBestTrajectory();
}

void ItompInterface::startMotionPlanning()
{
    if (!is_optimizing_)
    {
        is_optimizing_ = true;
        execution_timer_->start();
        optimizer_.startOptimizationThread();
    }
}

void ItompInterface::stopMotionPlanning()
{
    if (is_optimizing_)
    {
        is_optimizing_ = false;
        optimizer_.stopOptimizationThread();
        execution_timer_->stop();
    }
}

void ItompInterface::moveTrajectoryForwardOneTimestep()
{
    optimizer_.moveForwardOneTimestep();
}

}
