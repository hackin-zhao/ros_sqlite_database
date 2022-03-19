#if !defined(_DATABASE_SERVICE_HPP_)
#define _DATABASE_SERVICE_HPP_

#include <ros/ros.h>
#include "ros_sqlite_database/message_option.hpp"

namespace database
{
    template <typename ServiceT>
    class DatabaseService
    {
    public:
        using ServiceType = ServiceT;
        using RequestType = typename ServiceT::Request;
        using ResponseType = typename ServiceT::Response;

        DatabaseService(ros::NodeHandle &nh, const std::string &srv_name) : srv_name_(srv_name)
        {
            std::string db_path;
            nh.param<std::string>("database_path", db_path, "/work/catkin_ws/test.db");
            db_opt_.connect(db_path.c_str());
            db_opt_.create_datatable();
            server_ = nh.advertiseService(srv_name_, &DatabaseService::DBcallback, this);
        }

        ~DatabaseService() { server_.shutdown(); };

    protected:
        bool DBcallback(RequestType &req, ResponseType &res)
        {
            ROS_INFO_STREAM("receive req: " << req);

            if (req.option == RequestType::SELECT)
            {
                res.data = db_opt_.query();
            }
            else if (req.option == RequestType::INSERT)
            {
                db_opt_.insert(req.data);
            }
            else if (req.option == RequestType::UPDATE)
            {
                db_opt_.update(req.data);
            }
            else if (req.option == RequestType::DELETE)
            {
                db_opt_.delete_records();
            }
            else
            {
                ROS_WARN_STREAM("database not support option: " << req.option);
            }
        }

    private:
        const std::string srv_name_;
        ros::ServiceServer server_;
        ros_sqlite_database::DatabaseOption<decltype(std::declval<RequestType>().data)> db_opt_;
    };

} // namespace database

#endif // _DATABASE_SERVICE_HPP_
