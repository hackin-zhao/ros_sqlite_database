#include <ros/ros.h>
#include "ros_sqlite_database/database_service.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_sqlite_database");

    ros::NodeHandle nh("~");
    sqlite_key_t map_sqlite_key{"map_id"};
    database::DatabaseService<ros_sqlite_database::database_map> map_service(nh, "/database_map", map_sqlite_key);

    ros::spin();
    return 0;
}
