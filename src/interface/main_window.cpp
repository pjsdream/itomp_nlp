#include <itomp_nlp/interface/main_window.h>

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
    grey_->setDiffuseColor(Eigen::Vector4f(0.5, 0.5, 0.5, 1));

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

    renderer_->update();
}

}
