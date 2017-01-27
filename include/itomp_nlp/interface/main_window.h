#ifndef ITOMP_INTERFACE_MAIN_WINDOW_H
#define ITOMP_INTERFACE_MAIN_WINDOW_H


#include <QMainWindow>

#include <itomp_nlp/renderer/renderer.h>
#include <itomp_nlp/renderer/robot_renderer.h>

#include <itomp_nlp/interface/itomp_interface.h>


namespace itomp
{

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow();

    void addRobot(RobotModel* robot_model);
    void addRobotEntity(int robot_index);
    void setRobotEntity(int robot_index, int entity_id, RobotState* robot_state);
    
protected slots:

    void updateNextFrame();

private:

    Renderer* renderer_;

    ItompInterface* itomp_interface_;

    std::vector<RobotRenderer*> robot_renderers_;
    std::vector<int> robot_entities_;
};

}


#endif // ITOMP_INTERFACE_MAIN_WINDOW_H