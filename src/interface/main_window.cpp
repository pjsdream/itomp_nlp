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
        RenderingRobot* rendering_robot = new RenderingRobot(renderer_, robot_model);
        renderer_->addShape(rendering_robot);
        rendering_robots_.push_back(rendering_robot);
    }

    timer->start();
}

void MainWindow::updateNextFrame()
{
    // update robot trajectory to renderer
    Eigen::MatrixXd trajectory = itomp_interface_->getBestTrajectory();
    
    for (int i=0; i<trajectory.cols() / 2; i++)
    {
        Eigen::VectorXd optimizer_robot_trajectory = trajectory.col(i*2);

        RobotState robot_state(*itomp_interface_->getRobotState());
        for (int j=0; j<itomp_interface_->getActiveJointNames().size(); j++)
            robot_state.setPosition(itomp_interface_->getActiveJointNames()[j], optimizer_robot_trajectory(j));

        // add robots renderer
        rendering_robots_[i]->setRobotState(robot_state);
    }

    renderer_->update();
}

}
