#include <itomp_nlp/renderer/renderer_interface.h>
#include <QTimer>


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
}

void RendererInterface::updateNextFrame()
{
    renderer_->update();
}

void RendererInterface::addRobot(itomp_robot::RobotModel* robot_model)
{
    RobotRenderer* robot_renderer = new RobotRenderer(renderer_, robot_model);
    robot_renderers_.push_back(robot_renderer);

    // TODO: for now, directly add entity although robot state is not given
    robot_renderer->addRobotEntity();
}

}
