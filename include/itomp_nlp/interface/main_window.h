#ifndef ITOMP_INTERFACE_MAIN_WINDOW_H
#define ITOMP_INTERFACE_MAIN_WINDOW_H


#include <QMainWindow>

#include <itomp_nlp/renderer/renderer.h>
#include <itomp_nlp/renderer/robot_renderer.h>

#include <itomp_nlp/renderer/rendering_robot.h>
#include <itomp_nlp/renderer/rendering_box.h>
#include <itomp_nlp/renderer/rendering_kinect_points.h>

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
    std::vector<RenderingBox*> rendering_boxes_; // for robot obbs

    std::vector<RenderingShape*> rendering_static_obstacles_;
    std::vector<RenderingShape*> rendering_dynamic_obstacles_;

    RenderingKinectPoints* rendering_point_cloud_;
    RenderingBox* rendering_table_;

    Material* grey_;
    Material* brown_;

    ItompInterface* itomp_interface_;

    OptimizerRobot* forward_kinematics_;
};

}


#endif // ITOMP_INTERFACE_MAIN_WINDOW_H