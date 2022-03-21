#if !defined(_DATABASE_SERVICE_HPP_)
#define _DATABASE_SERVICE_HPP_

#include <ros/ros.h>
#include "ros_sqlite_database/message_option.hpp"
#include "twc_database/utility.hpp"

namespace database
{
    template <typename ServiceT>
    class DatabaseService
    {
    public:
        using ServiceType = ServiceT;
        using RequestType = typename ServiceT::Request;
        using ResponseType = typename ServiceT::Response;
        using Msg = decltype(std::declval<RequestType>().data);

        DatabaseService(ros::NodeHandle &nh, const std::string &srv_name) : srv_name_(srv_name)
        {
            std::string db_path;
            nh.param<std::string>("database_path", db_path, "/work/catkin_ws/test.db");
            db_opt_.connect(db_path.c_str());
            db_opt_.create_datatable();
            server_ = nh.advertiseService(srv_name_, &DatabaseService::DBcallback, this);
        }

        ~DatabaseService() { server_.shutdown(); };

        template <typename T>
        bool setParam(T &&value, int i, std::string &string_value)
        {
            using U = std::remove_const_t<std::remove_reference_t<T>>;
            if constexpr (std::is_integral_v<U>)
            {
                if (value != 0)
                    string_value = std::to_string(value);
                else
                    return false;
            }
            else if constexpr (std::is_floating_point_v<U>)
            {
                if (value != 0.0)
                    string_value = std::to_string(value);
                else
                    return false;
            }
            else if constexpr (std::is_same_v<std::string, U>)
            {
                if (!value.empty())
                    string_value = "'" + value + "'";
                else
                    return false;
            }
            else if constexpr (sqlite_wapper::is_char_array_v<U>)
            {
                if (sizeof(U))
                    string_value = "'" + std::string(value, sizeof(U)) + "'";
                else
                    return false;
            }
            else
            {
                return false;
            }
        }

    protected:
        bool DBcallback(RequestType &req, ResponseType &res)
        {
            ROS_INFO_STREAM("receive req: " << req);

            if (req.option == RequestType::SELECT)
            {
                Msg msg = req.data;
                std::string sql;

                int index = 0;
                iguana::for_each(msg, [&sql, &msg, &index, this](auto item, auto i)
                                 {
                    std::string key = iguana::get_name<Msg>(decltype(i)::value).data();
                    std::string value;
                    if (setParam(msg.*item, index+1, value))
                    {
                        sql += key + "=" + value + " and ";
                    }
                    index++; });

                if (sql.empty())
                {
                    res.data = db_opt_.query();
                }
                else
                {
                    sql = sql.substr(0, sql.length() - strlen("and "));
                    sql = " where " + sql;
                    res.data = db_opt_.query(sql);
                }
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

            return true;
        }

    private:
        const std::string srv_name_;
        ros::ServiceServer server_;
        ros_sqlite_database::DatabaseOption<Msg> db_opt_;
    };

} // namespace database

#endif // _DATABASE_SERVICE_HPP_
