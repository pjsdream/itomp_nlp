#ifndef ITOMP_RENDERER_ROBOT_TRAJECTORY_H
#define ITOMP_RENDERER_ROBOT_TRAJECTORY_H


#include <itomp_nlp/renderer/rendering_robot.h>
#include <itomp_nlp/optimization/optimizer_robot.h>
#include <itomp_nlp/optimization/trajectory.h>


namespace itomp
{

class RenderingRobotTrajectory : public RenderingShape
{
public:

    RenderingRobotTrajectory(Renderer* renderer, RobotModel* robot_model, const RobotState& robot_state);

    virtual void draw(ShaderProgram* shader);

    void attachObject(RenderingShape* object);

    void setForwardKinematics(OptimizerRobot* forward_kinematics);
    void setRobotTrajectory(const Trajectory& trajectory);
    void setTime(double time);
    
private:
    
    RobotModel* robot_model_;
    RobotState robot_state_;
    OptimizerRobot* forward_kinematics_;

    RenderingRobot rendering_robot_;

    RenderingShape* rendering_object_;

    Trajectory trajectory_;
    double time_;
};

}


#endif // ITOMP_RENDERER_ROBOT_TRAJECTORY_H