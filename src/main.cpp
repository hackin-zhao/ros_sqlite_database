#include <ros/ros.h>
#include "ros_sqlite_database/database_service.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_sqlite_database");

    ros::NodeHandle nh;
    database::DatabaseService<ros_sqlite_database::database_map> map_service(nh, "database_map");

    ros::spin();
    return 0;
}
