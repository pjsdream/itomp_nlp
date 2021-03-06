#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_H


#include <itomp_nlp/shape/shape.h>

#include <Eigen/Dense>

#include <vector>
#include <Eigen/StdVector>

#include <itomp_nlp/robot/robot_model.h>
#include <itomp_nlp/robot/robot_state.h>


namespace itomp
{

/** class OptimizerRobot
 *  Open-loop robot model that only has optimizing joints
 *  Link index starts at 0 (base link)
 *  Joint index starts at 1
 */
class OptimizerRobot
{
public:
    
    enum JointType
    {
        JOINT_TYPE_PRISMATIC = 0,
        JOINT_TYPE_REVOLUTE,
        JOINT_TYPE_FIXED,
        JOINT_TYPE_ROOT,
    };
    
    static const double position_lower_default_;
    static const double position_upper_default_;

    static const double velocity_lower_default_;
    static const double velocity_upper_default_;

    template<class T>
    using EigenAlignedVector = std::vector<T, Eigen::aligned_allocator<T> >;

    struct Link
    {
        // collision shapes
        // cloned for thread-safety
        std::vector<Shape*> shapes;
    };

    struct Joint
    {
        // parent index
        int parent;

        // joint info
        JointType joint_type;
        Eigen::Vector3d axis;
        Eigen::Affine3d origin;

        // joint limits
        double position_lower;
        double position_upper;
        double velocity_lower;
        double velocity_upper;
    };

public:

    OptimizerRobot();
    ~OptimizerRobot();
    
    // copy constructor for creating memory for collision shapes
    OptimizerRobot(const OptimizerRobot& robot);

    // clone
    inline OptimizerRobot* clone()
    {
        return new OptimizerRobot(*this);
    }

    inline int getDoF() const
    {
        return joints_.size() - 1;
    }

    inline int getNumLinks() const
    {
        return links_.size();
    }
    
    void setLinkJoints(const std::vector<Link>& links, const std::vector<Joint>& joints);

    void setPositions(const Eigen::VectorXd& positions);
    void setVelocities(const Eigen::VectorXd& velocities);

    void setBaseTransform(const Eigen::Affine3d& transform);

    /// do forward kinematics and store the results in cache
    void forwardKinematics();

    inline const std::vector<Shape*>& getCollisionShapes(int idx)
    {
        return fk_shapes_[idx];
    }

    inline const Eigen::Affine3d& getLinkWorldTransform(int idx)
    {
        return link_world_transforms_[idx];
    }

    bool needSelfCollisionChecking(int link_id1, int link_id2) const;

    Eigen::Vector3d getWorldVelocity(int link_idx, const Eigen::Vector3d& local_position);

private:

    std::vector<Link> links_;
    std::vector<Joint> joints_;

    Eigen::VectorXd positions_;
    Eigen::VectorXd velocities_;

    // forward kinematics
    EigenAlignedVector<Eigen::Affine3d> link_world_transforms_;

    // velocity forward kinematics
    EigenAlignedVector<Eigen::Matrix4d> link_world_transform_derivatives_;
    Eigen::Matrix4d prismaticJointDerivative(int joint_idx);
    Eigen::Matrix4d revoluteJointDerivative(int joint_idx);

    // fk transforms
    std::vector<std::vector<Shape*> > fk_shapes_;
};

}


#endif // ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_H
