#ifndef ITOMP_NLP_OPTIMIZATION_TRAJECTORY_H
#define ITOMP_NLP_OPTIMIZATION_TRAJECTORY_H


#include <vector>
#include <string>
#include <Eigen/Dense>


namespace itomp
{

class Trajectory
{
public:

    Trajectory();
    Trajectory(const std::vector<std::string>& joint_names, double duration, Eigen::MatrixXd trajectory);
    Trajectory(const std::string& serial);

    std::string serialize() const;

    inline const std::vector<std::string>& getJointNames() const
    {
        return joint_names_;
    }

    inline double getDuration() const
    {
        return duration_;
    }

    inline const Eigen::MatrixXd& getTrajectory() const
    {
        return trajectory_;
    }

private:

    std::vector<std::string> joint_names_;
    double duration_;
    Eigen::MatrixXd trajectory_;
};

}


#endif // ITOMP_NLP_OPTIMIZATION_TRAJECTORY_H
