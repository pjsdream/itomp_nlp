#include <itomp_nlp/optimization/optimizer_robot_loader.h>

#include <itomp_nlp/robot/link.h>

#include <itomp_nlp/robot/prismatic_joint.h>
#include <itomp_nlp/robot/revolute_joint.h>
#include <itomp_nlp/robot/continuous_joint.h>
#include <itomp_nlp/robot/fixed_joint.h>

#include <itomp_nlp/shape/mesh.h>


namespace itomp_optimization
{

OptimizerRobotLoader::OptimizerRobotLoader()
{
}

void OptimizerRobotLoader::addAABBList(const std::vector<std::string> aabb_list)
{
    aabb_lists_.push_back(aabb_list);
    is_aabb_encountered_.push_back(false);
    aabbs_.push_back(itomp_shape::AABB());
    aabb_link_id_.push_back(-1);
}

OptimizerRobot* OptimizerRobotLoader::loadRobot(itomp_robot::RobotModel* robot_model, itomp_robot::RobotState* robot_state, const std::vector<std::string>& active_joint_names)
{
    active_joint_names_ = active_joint_names;
    robot_state_ = robot_state;

    OptimizerRobot* robot = new OptimizerRobot();

    const itomp_robot::Link* root_link = robot_model->getRootLink();

    loadRobotRecursive(root_link, Eigen::Affine3d::Identity(), -1);

    // convert aabb to obb
    for (int i=0; i<aabbs_.size(); i++)
    {
        const int link_id = aabb_link_id_[i];
        const itomp_shape::AABB& aabb = aabbs_[i];

        itomp_shape::OBB* obb = new itomp_shape::OBB(aabb);
        optimizer_links_[ link_id ].shapes.push_back(obb);
    }
    
    robot->setLinkJoints(optimizer_links_, optimizer_joints_);

    return robot;
}

void OptimizerRobotLoader::loadRobotRecursive(const itomp_robot::Link* link, const Eigen::Affine3d& transform, int parent_id)
{
    itomp_robot::Joint* parent_joint = link->getParentJoint();
    Eigen::Affine3d current_transform;

    if (parent_joint == 0 ||
        std::find(active_joint_names_.begin(), active_joint_names_.end(), parent_joint->getJointName()) != active_joint_names_.end())
    {
        current_transform = Eigen::Affine3d::Identity();

        OptimizerRobot::Link optimizer_link;
        OptimizerRobot::Joint optimizer_joint;

        optimizer_joint.parent = parent_id;
        optimizer_joint.origin = transform;

        if (parent_joint == 0 || parent_joint->fixedJoint())
        {
            optimizer_joint.joint_type = OptimizerRobot::JOINT_TYPE_FIXED;
        }

        else if (parent_joint->continuousJoint())
        {
            const itomp_robot::ContinuousJoint* continuous_joint = parent_joint->continuousJoint();

            optimizer_joint.joint_type = OptimizerRobot::JOINT_TYPE_REVOLUTE;
            optimizer_joint.axis = continuous_joint->getAxis();
            optimizer_joint.position_lower = OptimizerRobot::position_lower_default_;
            optimizer_joint.position_upper = OptimizerRobot::position_upper_default_;
            optimizer_joint.velocity_lower = OptimizerRobot::velocity_lower_default_;
            optimizer_joint.velocity_upper = OptimizerRobot::velocity_upper_default_;
        }

        else if (parent_joint->revoluteJoint())
        {
            const itomp_robot::RevoluteJoint* revolute_joint = parent_joint->revoluteJoint();

            optimizer_joint.joint_type = OptimizerRobot::JOINT_TYPE_REVOLUTE;
            optimizer_joint.axis = revolute_joint->getAxis();
            optimizer_joint.position_lower = revolute_joint->getPositionLowerLimit();
            optimizer_joint.position_upper = revolute_joint->getPositionUpperLimit();
            optimizer_joint.velocity_lower = OptimizerRobot::velocity_lower_default_;
            optimizer_joint.velocity_upper = OptimizerRobot::velocity_upper_default_;
        }

        else if (parent_joint->prismaticJoint())
        {
            const itomp_robot::PrismaticJoint* prismatic_joint = parent_joint->prismaticJoint();

            optimizer_joint.joint_type = OptimizerRobot::JOINT_TYPE_PRISMATIC;
            optimizer_joint.axis = prismatic_joint->getAxis();
            optimizer_joint.position_lower = prismatic_joint->getPositionLowerLimit();
            optimizer_joint.position_upper = prismatic_joint->getPositionUpperLimit();
            optimizer_joint.velocity_lower = OptimizerRobot::velocity_lower_default_;
            optimizer_joint.velocity_upper = OptimizerRobot::velocity_upper_default_;
        }
        
        parent_id = optimizer_links_.size();

        optimizer_links_.push_back(optimizer_link);
        optimizer_joints_.push_back(optimizer_joint);
    }
    else
    {
        current_transform = transform;
    }

    // aabb
    for (int i=0; i<aabb_lists_.size(); i++)
    {
        const std::vector<std::string>& aabb_list = aabb_lists_[i];

        if (std::find(aabb_list.begin(), aabb_list.end(), link->getLinkName()) != aabb_list.end())
        {
            aabb_link_id_[i] = optimizer_links_.size() - 1;

            const std::vector<itomp_shape::Shape*> shapes = link->getCollisionShapes();
            itomp_shape::AABB& aabb = aabbs_[i];


            for (int j=0; j<shapes.size(); j++)
            {
                // TODO: collision shapes for other types than mesh

                itomp_shape::Mesh* mesh = dynamic_cast<itomp_shape::Mesh*>(shapes[j]);
                if (mesh != 0)
                {
                    itomp_shape::AABB shape_aabb = mesh->getAABB();

                    // apply translation
                    // TODO: apply orientation
                    shape_aabb.translate(transform.translation());

                    if (!is_aabb_encountered_[i])
                    {
                        is_aabb_encountered_[i] = true;
                        aabb = shape_aabb;
                    }
                    else
                        aabb = aabb.merge(shape_aabb);
                }
            }
        }
    }

    for (int i=0; i<link->getNumChild(); i++)
    {
        const itomp_robot::Joint* child_joint = link->getChildJoint(i);;
        const itomp_robot::Link* child_link = child_joint->getChildLink();

        Eigen::Affine3d child_transform = current_transform * child_joint->getOrigin() * child_joint->getTransform( robot_state_->getPosition(child_joint->getJointName()) );

        loadRobotRecursive(child_link, child_transform, parent_id);
    }
}

}