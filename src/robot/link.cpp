#include <itomp_nlp/robot/link.h>

#include <itomp_nlp/robot/joint.h>


namespace itomp_robot
{

Link::Link()
    : parent_joint_(0)
{
}

void Link::addVisualMesh(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const std::string& mesh_filename)
{
    Eigen::Affine3d m = Eigen::Affine3d::Identity();
    m.rotate(orientation);
    m.translate(position);
    visual_origins_.push_back(m);

    visual_mesh_filenames_.push_back(mesh_filename);
}

void Link::setParentJoint(Joint* joint)
{
    parent_joint_ = joint;
}

void Link::addChildJoint(Joint* joint)
{
    child_joints_.push_back(joint);
}

Link* Link::getChildLink(int idx) const
{
    return child_joints_[idx]->getChildLink();
}

}