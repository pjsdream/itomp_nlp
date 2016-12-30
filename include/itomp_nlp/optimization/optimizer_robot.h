#ifndef ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_H
#define ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_H


#include <itomp_nlp/shape/shape.h>

#include <Eigen/Dense>

#include <vector>
#include <Eigen/StdVector>


namespace itomp_optimization
{

/** class OptimizerRobot
 *  Open-loop robot model that only has optimizing joints
 *  Link index starts at 0 (base link)
 *  Joint index starts at 1
 */
class OptimizerRobot
{
private:

    enum JointType
    {
        JOINT_TYPE_PRISMATIC = 0,
        JOINT_TYPE_REVOLUTE,
    };

public:
    
    template<class T>
    using EigenAlignedVector = std::vector<T, Eigen::aligned_allocator<T> >;

    struct Link
    {
        // collision shapes
        // cloned for thread-safety
        std::vector<itomp_shape::Shape*> shapes;
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

    void setPositions(const Eigen::VectorXd& positions);
    void setVelocities(const Eigen::VectorXd& velocities);

    void setBaseTransform(const Eigen::Affine3d& transform);

    /// do forward kinematics and store the results in cache
    void forwardKinematics();

    inline const std::vector<itomp_shape::Shape*>& getCollisionShapes()
    {
        return fk_shapes_;
    }

private:

    std::vector<Link> links_;
    std::vector<Joint> joints_;

    Eigen::VectorXd positions_;
    Eigen::VectorXd velocities_;

    // forward kinematics
    EigenAlignedVector<Eigen::Affine3d> link_world_transforms_;

    // fk transforms
    std::vector<itomp_shape::Shape*> fk_shapes_;
};

}


#endif // ITOMP_OPTIMIZATION_OPTIMIZER_ROBOT_H