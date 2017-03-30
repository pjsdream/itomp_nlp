#ifndef ITOMP_INTERFACE_ITOMP_INTERFACE_H
#define ITOMP_INTERFACE_ITOMP_INTERFACE_H


#include <itomp_nlp/network/trajectory_publisher.h>

#include <QWidget>

#include <QGridLayout>
#include <QScrollArea>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>

#include <itomp_nlp/optimization/optimizer.h>
#include <itomp_nlp/optimization/optimizer_robot_loader.h>

#include <itomp_nlp/optimization/dynamic_kf_human_obstacle.h>

#include <itomp_nlp/interface/itomp_cost_functions_widget.h>
#include <itomp_nlp/interface/itomp_nlp_widget.h>


namespace itomp
{

class ItompInterface : public QWidget
{
    Q_OBJECT

public:

    ItompInterface(QWidget* parent = 0);
    
    inline RobotModel* getRobotModel()
    {
        return robot_model_;
    }

    inline RobotState* getRobotState()
    {
        return robot_state_;
    }

    inline OptimizerRobot* getOptimizerRobot()
    {
        return optimizer_robot_;
    }

    inline int getNumInterpolatedVariables()
    {
        return optimizer_.getNumInterpolatedVariables();
    }

    inline const std::vector<std::string>& getActiveJointNames()
    {
        return active_joint_names_;
    }

    inline Scene* getScene() const
    {
        return optimizer_.getScene();
    }

    // TODO: trajectory output type
    Eigen::MatrixXd getBestTrajectory();

public Q_SLOTS:

    void startMotionPlanning();
    void stopMotionPlanning();
    void resetMotionPlanning();
    void moveTrajectoryForwardOneTimestep();

    void costFunctionChanged(int id, const std::string& type, std::vector<double> values);

    void commandAdded(std::string command);

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
    QPushButton* start_button_;
    QPushButton* stop_button_;
    QPushButton* reset_button_;

    QScrollArea* scroll_area_;
    ItompCostFunctionsWidget* itomp_cost_functions_widget_;

    ItompNLPWidget* nlp_widget_;

    QTimer* execution_timer_;
};

}


#endif // ITOMP_INTERFACE_ITOMP_INTERFACE_H