#include <itomp_nlp/robot/urdf_parser.h>

#include <itomp_nlp/robot/revolute_joint.h>

#include <map>


namespace itomp_robot
{

URDFParser::URDFParser()
{
}

void URDFParser::addPackageDirectoryMapping(const std::string& package_name, const std::string& directory)
{
    package_map_[package_name] = directory;
}

RobotModel* URDFParser::parseURDF(const std::string& filename)
{
    tinyxml2::XMLDocument document;
    tinyxml2::XMLError error = document.LoadFile(filename.c_str());

    if (error != tinyxml2::XMLError::XML_SUCCESS)
    {
        fprintf(stderr, "URDF Parser xml parsing error: %s\n", document.ErrorName());
        return 0;
    }

    RobotModel* robot_model = new RobotModel();

    tinyxml2::XMLElement* robot_element = document.FirstChildElement("robot");
    robot_model->setRobotName( robot_element->Attribute("name") );

    // parse links
    std::map<std::string, Link*> link_map;
    tinyxml2::XMLElement* link_element = robot_element->FirstChildElement("link");
    while (link_element != 0)
    {
        Link* link = new Link();

        std::string link_name = link_element->Attribute("name");
        link->setLinkName(link_name);

        // TODO: intertial

        // visual
        tinyxml2::XMLElement* visual_element = link_element->FirstChildElement("visual");
        if (visual_element != 0)
        {
            // origin
            tinyxml2::XMLElement* origin_element = visual_element->FirstChildElement("origin");
            Eigen::Vector3d position;
            Eigen::Quaterniond orientation;
            parseOriginElement(origin_element, position, orientation);

            // geometry
            tinyxml2::XMLElement* geometry_element = visual_element->FirstChildElement("geometry");
            std::string mesh_filename;

            if (geometry_element != 0)
            {
                // mesh
                tinyxml2::XMLElement* mesh_element = geometry_element->FirstChildElement("mesh");

                if (mesh_element != 0)
                {
                    mesh_filename = mesh_element->Attribute("filename");

                    if (mesh_filename.substr(0, 10) == "package://")
                        resolvePackage(mesh_filename);
                }
            }

            if (mesh_filename != "")
                link->addVisualMesh(position, orientation, mesh_filename);
        }

        // TODO: collision

        link_map[link_name] = link;

        link_element = link_element->NextSiblingElement("link");
    }

    // parse joints
    std::map<std::string, Joint*> joint_map;
    std::map<std::string, std::string> parent_map;
    std::map<std::string, std::string> child_map;
    tinyxml2::XMLElement* joint_element = robot_element->FirstChildElement("joint");
    while (joint_element != 0)
    {
        Joint* joint;
        RevoluteJoint* revolute_joint;

        std::string joint_type = joint_element->Attribute("type");

        if (joint_type == "revolute")
            joint = revolute_joint = new RevoluteJoint();

        // TODO: other types
        // currently revolute joint for all types
        else
            joint = revolute_joint = new RevoluteJoint();
        
        std::string joint_name = joint_element->Attribute("name");
        joint->setJointName(joint_name);

        // origin
        tinyxml2::XMLElement* origin_element = joint_element->FirstChildElement("origin");
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        parseOriginElement(origin_element, position, orientation);
        joint->setOrigin(position, orientation);

        // parent
        std::string parent_name = joint_element->FirstChildElement("parent")->Attribute("link");

        // child
        std::string child_name = joint_element->FirstChildElement("child")->Attribute("link");

        // axis
        tinyxml2::XMLElement* axis_element = joint_element->FirstChildElement("axis");
        Eigen::Vector3d axis;
        parseAxisElement(axis_element, axis);

        if (revolute_joint) // TODO: other types of joints
            revolute_joint->setAxis(axis);

        // TODO: calibration
        // TODO: dynamics

        // limit
        tinyxml2::XMLElement* limit_element = joint_element->FirstChildElement("limit");
        double lower;
        double upper;
        parseLimitElement(limit_element, lower, upper);

        if (revolute_joint) // TODO: other types of joints
            revolute_joint->setLimit(lower, upper);

        // TODO: mimic
        // TODO: safety_controller

        joint_map[joint_name] = joint;
        parent_map[joint_name] = parent_name;
        child_map[joint_name] = child_name;

        joint_element = joint_element->NextSiblingElement("joint");
    }

    // make link-joint tree
    for (std::map<std::string, Joint*>::iterator it = joint_map.begin(); it != joint_map.end(); it++)
    {
        Joint* joint = it->second;

        const std::string parent_name = parent_map[it->first];
        const std::string child_name = child_map[it->first];

        Link* parent_link = link_map[parent_name];
        Link* child_link = link_map[child_name];

        parent_link->addChildJoint(joint);
        child_link->setParentJoint(joint);

        joint->setParentLink(parent_link);
        joint->setChildLink(child_link);
    }

    for (std::map<std::string, Link*>::iterator it = link_map.begin(); it != link_map.end(); it++)
    {
        Link* link = it->second;

        if (!link->hasParent())
        {
            robot_model->setRootLink(link);
            break;
        }
    }

    return robot_model;
}

void URDFParser::resolvePackage(std::string& filename)
{
    filename = filename.substr(10);

    const int idx = filename.find_first_of("/");
    const std::string package_name = filename.substr(0, idx);

    const std::map<std::string, std::string>::iterator it = package_map_.find(package_name);
    if (it != package_map_.end())
        filename.replace(0, idx, it->second);
}

void URDFParser::parseOriginElement(tinyxml2::XMLElement* origin_element, Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
{
    if (origin_element != 0)
    {
        const char* xyz = origin_element->Attribute("xyz");
        if (xyz)
            sscanf(xyz, "%lf%lf%lf", &position(0), &position(1), &position(2));

        const char* rpy = origin_element->Attribute("rpy");
        if (rpy)
        {
            double r, p, y;
            sscanf(rpy, "%lf%lf%lf", &r, &p, &y);
            const Eigen::Quaterniond qy(Eigen::AngleAxisd(y, Eigen::Vector3d(0, 0, 1)));
            const Eigen::Quaterniond qp(Eigen::AngleAxisd(p, Eigen::Vector3d(0, 1, 0)));
            const Eigen::Quaterniond qr(Eigen::AngleAxisd(r, Eigen::Vector3d(1, 0, 0)));
            orientation = qr * qp * qy;
        }
    }

    else
    {
        position = Eigen::Vector3d::Zero();
        orientation = Eigen::Quaterniond::Identity();
    }
}

void URDFParser::parseAxisElement(tinyxml2::XMLElement* axis_element, Eigen::Vector3d& axis)
{
    if (axis_element != 0)
    {
        const char* xyz = axis_element->Attribute("xyz");
        sscanf(xyz, "%lf%lf%lf", &axis(0), &axis(1), &axis(2));
    }

    else
    {
        axis = Eigen::Vector3d(1, 0, 0);
    }
}

void URDFParser::parseLimitElement(tinyxml2::XMLElement* limit_element, double& lower, double& upper)
{
    lower = 0.;
    upper = 0.;

    if (limit_element != 0)
    {
        lower = limit_element->DoubleAttribute("lower");
        upper = limit_element->DoubleAttribute("upper");
    }
}

}
