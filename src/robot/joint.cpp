#include <itomp_nlp/robot/joint.h>


namespace itomp_robot
{

Joint::Joint()
    : parent_link_(0)
    , child_link_(0)
{
}

void Joint::setParentLink(Link* link)
{
    parent_link_ = link;
}

void Joint::setChildLink(Link* link)
{
    child_link_ = link;
}

void Joint::setOrigin(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
    origin_.setIdentity();
    origin_.rotate(orientation);
    origin_.translate(position);
}

}
