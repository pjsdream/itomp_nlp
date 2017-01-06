#ifndef ITOMP_INTERFACE_ITOMP_INTERFACE_H
#define ITOMP_INTERFACE_ITOMP_INTERFACE_H


#include <QWidget>

#include <QGridLayout>
#include <QScrollArea>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>

#include <itomp_nlp/optimization/optimizer.h>
#include <itomp_nlp/optimization/optimizer_robot_loader.h>


namespace itomp_interface
{

class ItompInterface : public QWidget
{
    Q_OBJECT

public:

    ItompInterface(QWidget* parent = 0);
    
    inline itomp_robot::RobotModel* getRobotModel()
    {
        return robot_model_;
    }

    inline itomp_robot::RobotState* getRobotState()
    {
        return robot_state_;
    }

    inline int getNumInterpolatedVariables()
    {
        return optimizer_.getNumInterpolatedVariables();
    }

    inline const std::vector<std::string>& getActiveJointNames()
    {
        return active_joint_names_;
    }

    // TODO: trajectory output type
    Eigen::MatrixXd getBestTrajectory();

public slots:

    void startMotionPlanning();
    void stopMotionPlanning();
    void moveTrajectoryForwardOneTimestep();

private:

    // resources
    void initializeResources();
    itomp_robot::RobotModel* robot_model_;
    itomp_robot::RobotState* robot_state_;
    itomp_optimization::OptimizerRobot* optimizer_robot_;
    itomp_optimization::Optimizer optimizer_;
    std::vector<std::string> active_joint_names_;
    std::vector<std::vector<std::string> > aabb_lists_;

    bool is_optimizing_;

    QGridLayout* layout_;
    QPushButton* start_button_;
    QPushButton* stop_button_;

    QScrollArea* scroll_area_;

    QTimer* execution_timer_;
};

}


#endif // ITOMP_INTERFACE_ITOMP_INTERFACE_H