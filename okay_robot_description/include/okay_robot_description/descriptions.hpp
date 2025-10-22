#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

inline const std::string& get_package_dir()
{
    static const std::string package_dir
        = ament_index_cpp::get_package_share_directory("okay_robot_description");
    return package_dir;
}

inline const std::string& get_okay_robot_xml_path()
{
    static const std::string path = get_package_dir() + "/okay_robot/scene.xml";
    return path;
}