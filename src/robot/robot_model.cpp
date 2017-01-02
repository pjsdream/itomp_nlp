#include <itomp_nlp/robot/robot_model.h>

#include <itomp_nlp/robot/joint.h>


namespace itomp_robot
{

RobotModel::RobotModel()
{
}

RobotModel::~RobotModel()
{
    // TODO: delete links
}

void RobotModel::setRootLink(Link* link)
{
    // TODO: delete current root link

    root_link_ = link;

    // link/joint namename
    link_names_.clear();
    joint_names_.clear();
    map_link_.clear();
    map_joint_.clear();

    initializeJointLinkNames(link);
}

void RobotModel::initializeJointLinkNames(Link* link)
{
    link_names_.push_back(link->getLinkName());
    map_link_[link->getLinkName()] = link;

    for (int i=0; i<link->getNumChild(); i++)
    {
        Joint* joint = link->getChildJoint(i);
        Link* child_link = joint->getChildLink();

        joint_names_.push_back(joint->getJointName());
        map_joint_[joint->getJointName()] = joint;

        initializeJointLinkNames(child_link);
    }
}

}
