#ifndef ITOMP_INTERFACE_MAIN_WINDOW_H
#define ITOMP_INTERFACE_MAIN_WINDOW_H


#include <QMainWindow>

#include <itomp_nlp/renderer/renderer.h>
#include <itomp_nlp/renderer/robot_renderer.h>

#include <itomp_nlp/interface/itomp_interface.h>


namespace itomp_interface
{

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow();

    void addRobot(itomp_robot::RobotModel* robot_model);
    void addRobotEntity(int robot_index);
    void setRobotEntity(int robot_index, int entity_id, itomp_robot::RobotState* robot_state);
    
protected slots:

    void updateNextFrame();

private:

    itomp_renderer::Renderer* renderer_;

    ItompInterface* itomp_interface_;

    std::vector<itomp_renderer::RobotRenderer*> robot_renderers_;
    std::vector<int> robot_entities_;
};

}


#endif // ITOMP_INTERFACE_MAIN_WINDOW_H