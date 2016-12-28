#ifndef ITOMP_ROBOT_URDF_PARSER_H
#define ITOMP_ROBOT_URDF_PARSER_H


#include <itomp_nlp/robot/robot_model.h>

#include <string>
#include <map>

#include <tinyxml2.h>


namespace itomp_robot
{

class URDFParser
{
public:

    URDFParser();

    void addPackageDirectoryMapping(const std::string& package_name, const std::string& directory);

    RobotModel* parseURDF(const std::string& filename);

private:

    void resolvePackage(std::string& filename);

    void parseOriginElement(tinyxml2::XMLElement* origin_element, Eigen::Vector3d& position, Eigen::Quaterniond& orientation);
    void parseAxisElement(tinyxml2::XMLElement* axis_element, Eigen::Vector3d& axis);
    void parseLimitElement(tinyxml2::XMLElement* limit_element, double& lower, double& upper);

    std::map<std::string, std::string> package_map_;
};

}


#endif // ITOMP_ROBOT_URDF_PARSER_H