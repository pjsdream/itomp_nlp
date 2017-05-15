#define _USE_MATH_DEFINES
 
#include <itomp_nlp/interface/itomp_interface.h>

#include <itomp_nlp/robot/urdf_parser.h>

#include <itomp_nlp/shape/obb.h>

//#include <itomp_nlp/optimization/dynamic_kf_human_obstacle.h>

#include <itomp_nlp/utils/timing.h>


namespace itomp
{

ItompInterface::ItompInterface(QWidget* parent)
    : QWidget(parent)
    , is_optimizing_(false)
    , phase_(-1)
{
    setWindowTitle("Motion planner");

    resize(600, 600);

    layout_ = new QGridLayout(this);
    setLayout(layout_);

    start_button_ = new QPushButton("Start", this);
    connect(start_button_, SIGNAL(clicked()), this, SLOT(startMotionPlanning()));

    stop_button_ = new QPushButton("Stop", this);
    connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopMotionPlanning()));

    reset_button_ = new QPushButton("Reset", this);
    connect(reset_button_, SIGNAL(clicked()), this, SLOT(resetMotionPlanning()));

    layout_->addWidget(start_button_, 0, 0);
    layout_->addWidget(stop_button_, 0, 1);
    layout_->addWidget(reset_button_, 0, 2);

    itomp_cost_functions_widget_ = new ItompCostFunctionsWidget(this);
    connect(itomp_cost_functions_widget_, SIGNAL(costFunctionChanged(int, const std::string&, std::vector<double>)),
            this, SLOT(costFunctionChanged(int, const std::string&, std::vector<double>)));
    
    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidget(itomp_cost_functions_widget_);
    scroll_area_->setWidgetResizable(true);

    layout_->setRowStretch(1, 1);
    layout_->addWidget(scroll_area_, 1, 0, 1, 3);
    
    // text edit
    nlp_widget_ = new ItompNLPWidget(this);
    nlp_widget_->setSpeechIPAddress("152.23.13.219");
    //nlp_widget_->setSpeechIPAddress("localhost");
    connect(nlp_widget_, SIGNAL(commandAdded(std::string)), this, SLOT(commandAdded(std::string)));
    
    layout_->setRowStretch(2, 0);
    layout_->addWidget(nlp_widget_, 2, 0, 1, 3);

    // execution tmier
    execution_timer_ = new QTimer(this);
    execution_timer_->setInterval(500);
    connect(execution_timer_, SIGNAL(timeout()), this, SLOT(moveTrajectoryForwardOneTimestep()));

    // initialize resource
    initializeResources();
}

void ItompInterface::initializeResources()
{
#ifdef _WIN32
    URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "..\\..\\fetch_ros\\fetch_description");
    robot_model_ = urdf_parser.parseURDF("../urdf/fetch.urdf");
#else
    URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "/home/jaesungp/catkin_ws/src/fetch_ros/fetch_description");
    robot_model_ = urdf_parser.parseURDF("/home/jaesungp/catkin_ws/src/fetch_ros/fetch_description/robots/fetch.urdf");
#endif

    // default robot state
    robot_state_ = new RobotState(robot_model_);
    //robot_state_->setPosition("shoulder_pan_joint", 0.79);
    robot_state_->setPosition("torso_lift_joint", 0.35);
    robot_state_->setPosition("r_gripper_finger_joint", 0.05);
    robot_state_->setPosition("l_gripper_finger_joint", 0.05);

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

    OptimizerRobotLoader optimizer_robot_loader;

    for (int i=0; i<aabb_lists_.size(); i++)
        optimizer_robot_loader.addAABBList(aabb_lists_[i]);

    optimizer_robot_ = optimizer_robot_loader.loadRobot(robot_model_, robot_state_, active_joint_names_);

    OptimizerOptions options;
    options.trajectory_duration = 2.0;
    options.timestep = 0.5;
    options.num_waypoints = 4;
    options.num_waypoint_interpolations = 3;

    optimizer_.setRobot(optimizer_robot_);
    optimizer_.setOptions(options);
    optimizer_.prepare();

    Eigen::Matrix<double, 7, 1> position;
    Eigen::Matrix<double, 7, 1> velocity;
    position.setZero();
    velocity.setZero();
    optimizer_.setInitialRobotState(position, velocity);

    // end effector link id = 7
    //optimizer_.setGoalPosition(7, Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0.5, 0.5, 1));
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

    // table obstacle
    Eigen::Affine3d table_center;
    table_center.setIdentity();
    table_center.translate(Eigen::Vector3d(0.8, 0, 0.35));
    OBB* table = new OBB(1, 2, 0.7, table_center);

    StaticObstacle* table_obstacle = new StaticObstacle();
    table_obstacle->addShape(table);

    optimizer_.addStaticObstacle(table_obstacle);

    // H shape obstacle
    /*
    Eigen::Affine3d obstacle_center;
    obstacle_center.setIdentity();
    obstacle_center.translate(Eigen::Vector3d(0.9, 0, 0.8));
    OBB* shape1 = new OBB(0.5, 1, 0.1, obstacle_center);

    obstacle_center.setIdentity();
    obstacle_center.translate(Eigen::Vector3d(0.9, 0, 1.4));
    OBB* shape2 = new OBB(0.5, 1, 0.1, obstacle_center);

    obstacle_center.setIdentity();
    obstacle_center.translate(Eigen::Vector3d(0.9, 0, 1.1));
    OBB* shape3 = new OBB(0.5, 0.1, 0.6, obstacle_center);

    StaticObstacle* obstacle = new StaticObstacle();
    obstacle->addShape(shape1);
    obstacle->addShape(shape2);
    obstacle->addShape(shape3);

    optimizer_.addStaticObstacle(obstacle);
    */

    // kinect camera
    /*
    Eigen::Affine3d camera_transform;
    camera_transform.setIdentity();
    camera_transform.translate(Eigen::Vector3d(-0.5, 0, 1.2));
    camera_transform.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)));
    camera_transform.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0, 0)));

    for (int i=0; i<KinectDevice::bodyCount(); i++)
    {
        DynamicKFHumanObstacle* human_obstacle = new DynamicKFHumanObstacle(i);
        human_obstacle->setCameraTransform(camera_transform);
        optimizer_.addDynamicObstacle(human_obstacle);
        human_obstacles_.push_back(human_obstacle);
    }
    */
}

Trajectory ItompInterface::getCurrentTrajectory()
{
    return trajectory_;
}

double ItompInterface::getCurrentTrajectoryTime()
{
    return getWallTime() - start_time_;
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

        start_time_ = getWallTime();
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

void ItompInterface::resetMotionPlanning()
{
    if (is_optimizing_)
    {
        optimizer_.stopOptimizationThread();
        execution_timer_->stop();

        optimizer_.resetWaypoints();

        execution_timer_->start();
        optimizer_.startOptimizationThread();
    }
    else
    {
        optimizer_.resetWaypoints();
    }
}

void ItompInterface::moveTrajectoryForwardOneTimestep()
{
    /*
    for (int i=0; i<human_obstacles_.size(); i++)
        human_obstacles_[i]->update();
        */

    // publish trajectory
    Eigen::MatrixXd trajectory_matrix = optimizer_.getBestTrajectory();
    Trajectory trajectory(active_joint_names_, 2.0, trajectory_matrix);
    trajectory_publisher_.publish(trajectory);

    optimizer_.moveForwardOneTimestep();
    optimizer_.updateScene();

    // change goal cost when reached to the goal
    /*
    const double threshold = 0.1;
    if (optimizer_.getBestTrajectoryCost() <= threshold)
        optimizer_.changeGoalCost();
        */

    // store current trajectory
    start_time_ = getWallTime();
    trajectory_ = trajectory;
}

void ItompInterface::costFunctionChanged(int id, const std::string& type, std::vector<double> values)
{
    if (type == "zero")
        optimizer_.setZeroCost(id);

    else if (type == "smoothness")
        optimizer_.setSmoothnessCost(id, values[0]);

    else if (type == "collision")
        optimizer_.setCollisionCost(id, values[0]);

    else if (type == "goal")
    {
        // hard-coded enffector information for fetch robot
        optimizer_.setGoalCost(id, values[0], 7, Eigen::Vector3d(values[1], values[2], values[3]), Eigen::Vector3d(values[4], values[5], values[6]));
    }

    else if (type == "orientation")
    {
        Eigen::Quaterniond q(values[1], values[2], values[3], values[4]);
        if (q.squaredNorm() < 1e-8)
            q.setIdentity();

        // hard-coded enffector information for fetch robot
        optimizer_.setGoalOrientationCost(id, values[0], 7, q);
    }

    else if (type == "upvector")
        optimizer_.setGoalUpvectorCost(id, values[0], 7, Eigen::Vector3d(values[1], values[2], values[3]));

    else if (type == "velocity")
        optimizer_.setVelocityCost(id, values[0], 7, Eigen::Vector3d(values[1], values[2], values[3]));
}

void ItompInterface::commandAdded(std::string command)
{
    printf("command: %s\n", command.c_str());
    //optimizer_.changeGoalCost();

    /*
    if (command.find("start") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(1.5, 0, 1.5));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("place it on the table") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 0, 0.78));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }
    else if (command.find("stop") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 0, 0.78));
        optimizer_.setGoalUpvectorCost(3, 0.0, 7, Eigen::Vector3d(0, 0, 1));
    }
    else if (command.find("don't put it") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 3);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 0.3, 0.78));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }
    */

    if (command.find("start") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(1.5, 0, 1.5));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
        phase_ = -1;
    }
    if (command.find("pick up this") != std::string::npos)
    {
        if (phase_ == -1)
        {
            optimizer_.setSmoothnessCost(0, 10);
            optimizer_.setCollisionCost(1, 30.0);
            optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(1.5, 0, 1.5));
            optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
            phase_ = 0;
        }
        else if (phase_ == 0)
        {
        }
    }

    /*
    if (command.find("don't") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 0.3);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, -0.3, 0.78));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("place") != std::string::npos ||
             command.find("put") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 0.3);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 0.0, 0.78));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("a") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 0.8, 1.4));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("b") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 0.6, 0.78));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("c") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(1.0, 0.1, 0.78));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("stop") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 0.8, 0.9));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    // H shape
    else if (command.find("hs") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.8, 0.3, 1.1));
    }

    else if (command.find("hg") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.8, -0.3, 1.1));
    }

    else if (command.find("h1") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 1.2, 1.1));
    }

    else if (command.find("h2") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, 1.2, 1.8));
    }

    else if (command.find("h3") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, -1.2, 1.8));
    }

    else if (command.find("h4") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.9, -1.2, 1.1));
    }

    else if (command.find("p11") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 10);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.3, 0, 0), Eigen::Vector3d(0.75, 0.75, 0.85));
        //optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("p12") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 1);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.75, 0.75, 0.75));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("p21") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 3);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.3, 0, 0), Eigen::Vector3d(0.75, -0.75, 0.85));
        //optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }

    else if (command.find("p22") != std::string::npos)
    {
        optimizer_.setSmoothnessCost(0, 1);
        optimizer_.setCollisionCost(1, 30.0);
        optimizer_.setGoalCost(2, 1.0, 7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.75, -0.75, 0.75));
        optimizer_.setGoalUpvectorCost(3, 10.0, 7, Eigen::Vector3d(0, 0, 1));
    }
    */
}

}
