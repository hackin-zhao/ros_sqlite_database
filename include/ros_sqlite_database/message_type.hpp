#if !defined(_MESSAGE_TYPE_HPP_)
#define _MESSAGE_TYPE_HPP_

#include "ros_sqlite_database/reflection.hpp"
#include <ros_sqlite_database/database_map.h>

namespace ros_sqlite_database
{
    REFLECTION(map, name, type, resolution, width, height, origin_x, origin_y, data)

} // namespace ros_sqlite_database

#endif // _MESSAGE_TYPE_HPP_
