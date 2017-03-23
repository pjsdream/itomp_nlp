#ifndef ITOMP_RENDERER_ROBOT_H
#define ITOMP_RENDERER_ROBOT_H


#include <itomp_nlp/renderer/rendering_shape.h>
#include <itomp_nlp/renderer/rendering_capsule.h>

#include <itomp_nlp/robot/robot_model.h>
#include <itomp_nlp/robot/robot_state.h>

#include <set>
#include <vector>

#include <Eigen/Dense>


namespace itomp
{

class RenderingRobot : public RenderingShape
{
public:

    RenderingRobot(Renderer* renderer, RobotModel* robot_model);

    virtual void draw(ShaderProgram* shader);

    void setRobotState(const RobotState& robot_state);
    
private:
    
    RobotModel* robot_model_;
    RobotState robot_state_;

    void drawLinks(const Link* link, Eigen::Affine3d transform);

    // intermediate variables
    ShaderProgram* shader_;
};

}


#endif // ITOMP_RENDERER_ROBOT_H