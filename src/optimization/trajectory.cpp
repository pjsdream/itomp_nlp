#include <itomp_nlp/optimization/trajectory.h>


namespace itomp
{

Trajectory::Trajectory()
{
}

Trajectory::Trajectory(const std::vector<std::string>& joint_names, double duration, Eigen::MatrixXd trajectory)
{
    joint_names_ = joint_names;
    duration_ = duration;
    trajectory_ = trajectory;
}

Trajectory::Trajectory(const std::string& serial)
{
    std::istringstream iss(serial);

    int n;
    iss >> n;
    joint_names_.resize(n);
    for (int i=0; i<n; i++)
        iss >> joint_names_[i];

    iss >> duration_;

    int rows, cols;
    iss >> rows >> cols;
    trajectory_.resize(rows, cols);
    for (int i=0; i<rows; i++)
        for (int j=0; j<cols; j++)
            iss >> trajectory_(i, j);
}

std::string Trajectory::serialize() const
{
    std::ostringstream oss;

    oss << joint_names_.size() << ' ';
    for (int i=0; i<joint_names_.size(); i++)
        oss << joint_names_[i] << ' ';

    oss << duration_ << ' ';

    oss << trajectory_.rows() << ' ' << trajectory_.cols() << ' ';
    for (int i=0; i<trajectory_.rows(); i++)
        for (int j=0; j<trajectory_.cols(); j++)
            oss << trajectory_(i, j) << ' ';

    return oss.str();
}

}
