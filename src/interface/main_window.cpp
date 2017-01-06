#include <itomp_nlp/interface/main_window.h>

#include <QTimer>


namespace itomp_interface
{

MainWindow::MainWindow()
    : QMainWindow()
{
    setWindowTitle("Motion planner");

    resize(800, 600);

    // central visualizer widget setup
    renderer_ = new itomp_renderer::Renderer(this);
    setCentralWidget(renderer_);
    show();

    // motion planner interface setup
    itomp_interface_ = new ItompInterface();
    itomp_interface_->move( pos().x() - itomp_interface_->width(), pos().y() );

    // timer
    QTimer* timer = new QTimer(this);
    timer->setInterval(16);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateNextFrame()));

    // renderer robots
    addRobot( itomp_interface_->getRobotModel() );
    const int num_interpolated_variables = itomp_interface_->getNumInterpolatedVariables();
    for (int i=0; i<num_interpolated_variables; i++)
        addRobotEntity(0);

    timer->start();
}

void MainWindow::updateNextFrame()
{
    // update robot trajectory to renderer
    Eigen::MatrixXd trajectory = itomp_interface_->getBestTrajectory();
    
    for (int i=0; i<trajectory.cols() / 2; i++)
    {
        Eigen::VectorXd optimizer_robot_trajectory = trajectory.col(i*2);

        itomp_robot::RobotState robot_state(*itomp_interface_->getRobotState());
        for (int j=0; j<itomp_interface_->getActiveJointNames().size(); j++)
            robot_state.setPosition(itomp_interface_->getActiveJointNames()[j], optimizer_robot_trajectory(j));

        setRobotEntity(0, i, &robot_state);
    }

    renderer_->update();
}

void MainWindow::addRobot(itomp_robot::RobotModel* robot_model)
{
    itomp_renderer::RobotRenderer* robot_renderer = new itomp_renderer::RobotRenderer(renderer_, robot_model);
    robot_renderers_.push_back(robot_renderer);
}

void MainWindow::addRobotEntity(int robot_index)
{
    robot_entities_.push_back( robot_renderers_[robot_index]->addRobotEntity() );
}

void MainWindow::setRobotEntity(int robot_index, int entity_id, itomp_robot::RobotState* robot_state)
{
    robot_renderers_[robot_index]->setRobotEntity(robot_entities_[entity_id], robot_state);
}

}
