#ifndef ITOMP_INTERFACE_ITOMP_DATAGEN_WIDGET_H
#define ITOMP_INTERFACE_ITOMP_DATAGEN_WIDGET_H


#include <itomp_nlp/network/trajectory_publisher.h>

#include <QWidget>

#include <QGridLayout>
#include <QScrollArea>
#include <QPushButton>
#include <QScrollArea>

#include <itomp_nlp/optimization/optimizer.h>
#include <itomp_nlp/optimization/optimizer_robot_loader.h>

#include <itomp_nlp/optimization/dynamic_kf_human_obstacle.h>

#include <itomp_nlp/interface/itomp_cost_functions_widget.h>
#include <itomp_nlp/interface/itomp_nlp_widget.h>


namespace itomp
{

class ItompDatagenWidget : public QWidget
{
    Q_OBJECT

public:

    ItompDatagenWidget(QWidget* parent = 0);
    
public Q_SLOTS:

    void resetMotionPlanning();
    void moveTrajectoryForwardOneTimestep();

private:

    // resources
    void initializeResources();
    RobotModel* robot_model_;
    RobotState* robot_state_;
    OptimizerRobot* optimizer_robot_;
    Optimizer optimizer_;
    std::vector<std::string> active_joint_names_;
    std::vector<std::vector<std::string> > aabb_lists_;
    std::vector<DynamicKFHumanObstacle*> human_obstacles_;

    bool is_optimizing_;

    TrajectoryPublisher trajectory_publisher_;

    QGridLayout* layout_;
    QPushButton* random_pose_button_;

    QScrollArea* scroll_area_;
    ItompCostFunctionsWidget* itomp_cost_functions_widget_;
};

}


#endif // ITOMP_INTERFACE_ITOMP_DATAGEN_WIDGET_H