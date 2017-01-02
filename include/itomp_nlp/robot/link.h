#ifndef ITOMP_ROBOT_LINK_H
#define ITOMP_ROBOT_LINK_H


#include <string>
#include <vector>
#include <Eigen/Dense>


namespace itomp_robot
{

class Joint;

/* Link
 *  Currently only supports visual meshes
 */
class Link
{
public:

    Link();

    inline bool hasParent() const
    {
        return parent_joint_ != 0;
    }

    inline void setLinkName(const std::string& name)
    {
        link_name_ = name;
    }

    inline const std::string& getLinkName() const
    {
        return link_name_;
    }

    inline const std::vector<std::string>& getVisualMeshFilenames() const
    {
        return visual_mesh_filenames_;
    }

    inline const std::vector<Eigen::Affine3d>& getVisualOrigins() const
    {
        return visual_origins_;
    }

    inline int getNumChild() const
    {
        return child_joints_.size();
    }

    Link* getChildLink(int idx) const;
    Joint* getChildJoint(int idx) const;
    Joint* getParentJoint() const;

    void addVisualMesh(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const std::string& mesh_filename);

    void setParentJoint(Joint* joint);
    void addChildJoint(Joint* joint);

private:

    std::string link_name_;

    // joints
    Joint* parent_joint_;
    std::vector<Joint*> child_joints_;

    // inertial

    // visual
    std::vector<Eigen::Affine3d> visual_origins_;
    std::vector<std::string> visual_mesh_filenames_;

    // collision
};

}


#endif // ITOMP_ROBOT_LINK_H