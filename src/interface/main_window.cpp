#include <itomp_nlp/interface/main_window.h>

#include <itomp_nlp/shape/capsule2.h>

#include <QTimer>


namespace itomp
{

MainWindow::MainWindow()
    : QMainWindow()
{
    setWindowTitle("Viewer");

    resize(800, 600);

    // central visualizer widget setup
    renderer_ = new Renderer(this);
    setCentralWidget(renderer_);
    show();

    // motion planner interface setup
    itomp_interface_ = new ItompInterface();
    itomp_interface_->show();

    // windows position
    itomp_interface_->move(100, 100);
    move( itomp_interface_->pos().x() + itomp_interface_->width(), itomp_interface_->pos().y() );

    // timer
    QTimer* timer = new QTimer(this);
    timer->setInterval(33);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateNextFrame()));

    // rendering
    const int num_interpolated_variables = itomp_interface_->getNumInterpolatedVariables();
    RobotModel* robot_model = itomp_interface_->getRobotModel();

    for (int i=0; i<num_interpolated_variables; i++)
    {
        //RenderingRobot* rendering_robot = new RenderingRobot(renderer_, robot_model);
        //rendering_robots_.push_back(rendering_robot);
    }

    forward_kinematics_ = itomp_interface_->getOptimizerRobot();

    grey_ = new Material();
    grey_->setDiffuseColor(Eigen::Vector4f(0.8, 0.8, 0.8, 1));

    timer->start();
}

void MainWindow::updateNextFrame()
{
    // update robot trajectory to renderer
    Eigen::MatrixXd trajectory = itomp_interface_->getBestTrajectory();
    
    int box_idx = 0;
    for (int i=0; i<trajectory.cols() / 2; i++)
    {
        Eigen::VectorXd optimizer_robot_trajectory = trajectory.col(i*2);

        RobotState robot_state(*itomp_interface_->getRobotState());
        for (int j=0; j<itomp_interface_->getActiveJointNames().size(); j++)
            robot_state.setPosition(itomp_interface_->getActiveJointNames()[j], optimizer_robot_trajectory(j));

        // update robot state to renderer
        //rendering_robots_[i]->setRobotState(robot_state);

        // update collision boxes
        forward_kinematics_->setPositions(trajectory.col(i*2));
        forward_kinematics_->setVelocities(trajectory.col(i*2+1));
        forward_kinematics_->forwardKinematics();

        const int num_links = forward_kinematics_->getNumLinks();
        for (int j=0; j<num_links; j++)
        {
            const std::vector<Shape*>& shapes = forward_kinematics_->getCollisionShapes(j);

            for (int k=0; k<shapes.size(); k++)
            {
                Shape* shape = shapes[k];

                OBB* obb = dynamic_cast<OBB*>(shape);
                if (obb != 0)
                {
                    if (box_idx >= rendering_boxes_.size())
                        rendering_boxes_.push_back(new RenderingBox(renderer_));

                    rendering_boxes_[box_idx]->setMaterial(grey_);
                    rendering_boxes_[box_idx]->setSize(obb->getSize());
                    rendering_boxes_[box_idx]->setTransform(obb->getTransform());
                    box_idx++;
                }
            }
        }
    }

    // remove unused rendering boxes
    while (box_idx < rendering_boxes_.size())
    {
        delete *rendering_boxes_.rbegin();
        rendering_boxes_.pop_back();
    }

    // update scene rendering
    const Scene* scene = itomp_interface_->getScene();
    const std::vector<StaticObstacle*> static_obstacles = scene->getStaticObstacles();

    int static_obstacle_idx = 0;
    for (int i=0; i<static_obstacles.size(); i++)
    {
        const StaticObstacle* static_obstacle = static_obstacles[i];
        const std::vector<Shape*> shapes = static_obstacle->getShapes();

        for (int j=0; j<shapes.size(); j++)
        {
            const Shape* shape = shapes[j];

            const OBB* obb = dynamic_cast<const OBB*>(shape);
            if (obb != 0)
            {
                if (static_obstacle_idx >= rendering_static_obstacles_.size())
                    rendering_static_obstacles_.push_back(new RenderingBox(renderer_));

                RenderingBox* rendering_box = dynamic_cast<RenderingBox*>(rendering_static_obstacles_[static_obstacle_idx]);
                rendering_box->setMaterial(grey_);
                rendering_box->setSize(obb->getSize());
                rendering_box->setTransform(obb->getTransform());
                static_obstacle_idx++;
            }
        }
    }

    // remove unused rendering static obstacles
    while (static_obstacle_idx < rendering_static_obstacles_.size())
    {
        delete *rendering_static_obstacles_.rbegin();
        rendering_static_obstacles_.pop_back();
    }

    // dynamic obstacles
    const std::vector<DynamicObstacle*> dynamic_obstacles = scene->getDynamicObstacles();

    int dynamic_obstacle_idx = 0;
    for (int i=0; i<dynamic_obstacles.size(); i++)
    {
        const double t = 0.5;

        DynamicObstacle* dynamic_obstacle = dynamic_obstacles[i];
        const std::vector<Shape*> shapes = dynamic_obstacle->getShapes(t);

        for (int j=0; j<shapes.size(); j++)
        {
            const Shape* shape = shapes[j];

            const Capsule2* capsule = dynamic_cast<const Capsule2*>(shape);
            if (capsule != 0)
            {
                if (dynamic_obstacle_idx >= rendering_dynamic_obstacles_.size())
                    rendering_dynamic_obstacles_.push_back(new RenderingCapsule(renderer_));

                RenderingCapsule* rendering_capsule = dynamic_cast<RenderingCapsule*>(rendering_dynamic_obstacles_[dynamic_obstacle_idx]);
                rendering_capsule->setMaterial(grey_);
                rendering_capsule->setCapsule(capsule->getP(), capsule->getRp(), capsule->getQ(), capsule->getRq());
                dynamic_obstacle_idx++;
            }
        }
    }

    // remove unused rendering dynamic obstacles
    while (dynamic_obstacle_idx < rendering_dynamic_obstacles_.size())
    {
        delete *rendering_dynamic_obstacles_.rbegin();
        rendering_dynamic_obstacles_.pop_back();
    }

    renderer_->update();
}

}
