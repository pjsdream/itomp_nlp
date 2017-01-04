#include <itomp_nlp/renderer/renderer_interface.h>
#include <QTimer>


#include <itomp_nlp/robot/urdf_parser.h>


namespace itomp_renderer
{

RendererInterface::RendererInterface()
{
    resize(800, 600);

    // central visualizer widget setup
    renderer_ = new Renderer(this);
    setCentralWidget(renderer_);
    show();

    // timer
    QTimer* timer = new QTimer(this);
    timer->setInterval(16);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateNextFrame()));
    timer->start();

    initializeResources();
}

void RendererInterface::initializeResources()
{
#ifdef WIN32
    itomp_robot::URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "..");
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
    optimizer_.setGoalPosition(7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0.5, 0.5, 1));
    optimizer_.setGoalVelocity(7, Eigen::Vector3d(0.2, 0, 0), Eigen::Vector3d(0, 0, 1));

    optimizer_.startOptimizationThread();

    // renderer robots
    addRobot(robot_model_);
    const int num_interpolated_variables = optimizer_.getNumInterpolatedVariables();
    for (int i=0; i<num_interpolated_variables; i++)
        addRobotEntity(0);
}

void RendererInterface::updateNextFrame()
{
    // update robot trajectory to renderer
    Eigen::MatrixXd trajectory = optimizer_.getBestTrajectory();
    
    for (int i=0; i<trajectory.cols() / 2; i++)
    {
        Eigen::VectorXd optimizer_robot_trajectory = trajectory.col(i*2);

        itomp_robot::RobotState robot_state(*robot_state_);
        for (int j=0; j<active_joint_names_.size(); j++)
            robot_state.setPosition(active_joint_names_[j], optimizer_robot_trajectory(j));

        setRobotEntity(0, i, &robot_state);
    }

    renderer_->update();
}

void RendererInterface::addRobot(itomp_robot::RobotModel* robot_model)
{
    RobotRenderer* robot_renderer = new RobotRenderer(renderer_, robot_model);
    robot_renderers_.push_back(robot_renderer);
}

void RendererInterface::addRobotEntity(int robot_index)
{
    robot_entities_.push_back( robot_renderers_[robot_index]->addRobotEntity() );
}

void RendererInterface::setRobotEntity(int robot_index, int entity_id, itomp_robot::RobotState* robot_state)
{
    robot_renderers_[robot_index]->setRobotEntity(robot_entities_[entity_id], robot_state);
}

}
