#ifndef ITOMP_INTERFACE_MAIN_WINDOW_H
#define ITOMP_INTERFACE_MAIN_WINDOW_H


#include <QMainWindow>

#include <itomp_nlp/renderer/renderer.h>
#include <itomp_nlp/renderer/robot_renderer.h>

#include <itomp_nlp/renderer/rendering_robot.h>

#include <itomp_nlp/interface/itomp_interface.h>


namespace itomp
{

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow();

protected slots:

    void updateNextFrame();

private:

    Renderer* renderer_;

    std::vector<RenderingRobot*> rendering_robots_;

    ItompInterface* itomp_interface_;
};

}


#endif // ITOMP_INTERFACE_MAIN_WINDOW_H