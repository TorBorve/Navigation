#pragma once

#include <nav_msgs/Path.h>
#include <string>

namespace file{

void savePath(const nav_msgs::Path& path, std::string filename);

void loadPath(nav_msgs::Path& path, std::string filename);

}