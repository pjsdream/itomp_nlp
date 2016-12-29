#include <QApplication>
#include <cstdlib>
#include <stdio.h>

#include <itomp_nlp/renderer/renderer_interface.h>

#include <itomp_nlp/robot/urdf_parser.h>

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    QApplication app(argc, argv);
    itomp_renderer::RendererInterface* renderer_interface = new itomp_renderer::RendererInterface();

    itomp_robot::URDFParser urdf_parser;
    urdf_parser.addPackageDirectoryMapping("fetch_description", "..");
    itomp_robot::RobotModel* robot_model = urdf_parser.parseURDF("../urdf/fetch.urdf");

    renderer_interface->addRobot(robot_model);
    renderer_interface->addRobotEntity(0);

    renderer_interface->show();

    app.exec();

    return 0;
}
