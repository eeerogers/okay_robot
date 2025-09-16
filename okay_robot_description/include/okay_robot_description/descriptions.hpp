#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

inline const std::string& get_package_dir()
{
    static const std::string package_dir
        = ament_index_cpp::get_package_share_directory("okay_robot_description");
    return package_dir;
}

inline const std::string& get_test_xml_path()
{
    static const std::string path = get_package_dir() + "/scene.xml";
    return path;
}

inline const std::string& get_okay_robot_xml_path()
{
    static const std::string path = get_package_dir() + "/okay_robot/scene.xml";
    return path;
}

inline const std::string& get_jaco3_xml_path()
{
    static const std::string path = get_package_dir() + "/kinova_gen3/scene.xml";
    return path;
}

inline const std::string& get_lerobot_xml_path()
{
    static const std::string path = get_package_dir() + "/low_cost_robot_arm/scene.xml";
    return path;
}

inline const std::string& get_tidybot_xml_path()
{
    static const std::string path = get_package_dir() + "/stanford_tidybot/scene.xml";
    return path;
}

inline const std::string& get_unitree_g1_xml_path()
{
    static const std::string path = get_package_dir() + "/unitree_g1/scene.xml";
    return path;
}
