#pragma once

#include <nav_msgs/Path.h>
#include <string>

namespace file{

/// @brief saves path to file.
/// @param[in] path path message that should be saved.
/// @param[in] filename filename path is saved to.
void savePath(const nav_msgs::Path& path, std::string filename);

/// @brief reads path from file.
/// @param[out] path path saved into this path message.
/// @param[in] filename filename path is read from.
void readPath(nav_msgs::Path& path, std::string filename);

}