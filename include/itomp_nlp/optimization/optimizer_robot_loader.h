#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_LOADER_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_LOADER_H


#include <itomp_nlp/optimization/optimizer_robot.h>

#include <itomp_nlp/shape/aabb.h>
#include <itomp_nlp/shape/obb.h>


namespace itomp
{

class OptimizerRobotLoader
{
public:

    OptimizerRobotLoader();

    void addAABBList(const std::vector<std::string>& aabb_list);

    inline void setAABBOffset(double offset)
    {
        aabb_offset_ = Eigen::Vector3d(offset, offset, offset);
    }

    OptimizerRobot* loadRobot(RobotModel* robot_model, RobotState* robot_state, const std::vector<std::string>& active_joint_names);

private:

    void loadRobotRecursive(const Link* link, const Eigen::Affine3d& transform, int parent_id);

    std::vector<std::vector<std::string> > aabb_lists_;
    std::vector<AABB> aabbs_;
    Eigen::Vector3d aabb_offset_;
    std::vector<char> is_aabb_encountered_;
    std::vector<int> aabb_link_id_;

    std::vector<std::string> active_joint_names_;
    RobotState* robot_state_;

    std::vector<OptimizerRobot::Link> optimizer_links_;
    std::vector<OptimizerRobot::Joint> optimizer_joints_;
};

}


#endif // ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_LOADER_H