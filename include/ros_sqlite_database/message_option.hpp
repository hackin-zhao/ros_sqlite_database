#if !defined(_MESSAGE_OPTION_HPP_)
#define _MESSAGE_OPTION_HPP_

#include "ros_sqlite_database/sqlite.hpp"
#include "ros_sqlite_database/dbng.hpp"
#include "ros_sqlite_database/message_type.hpp"

namespace ros_sqlite_database
{
    template <typename T>
    class DatabaseOption
    {
    private:
        sqlite_wapper::dbng<sqlite_wapper::sqlite> sqlite;

    public:
        DatabaseOption(){};
        ~DatabaseOption(){};

        template <typename... Args>
        bool connect(Args &&...args)
        {
            return sqlite.connect(std::forward<Args>(args)...);
        }

        template <typename... Args>
        bool create_datatable(Args &&...args)
        {
            return sqlite.create_datatable<T>(std::forward<Args>(args)...);
        }

        template <typename... Args>
        std::vector<T> query(Args &&...args)
        {
            return sqlite.query<T>(std::forward<Args>(args)...);
        }

        template <typename... Args>
        int insert(const T &t, Args &&...args)
        {
            return sqlite.insert(t, std::forward<Args>(args)...);
        }

        template <typename... Args>
        int insert(const std::vector<T> &t, Args &&...args)
        {
            return sqlite.insert(t, std::forward<Args>(args)...);
        }

        template <typename... Args>
        int update(const T &t, Args &&...args)
        {
            return sqlite.update(t, std::forward<Args>(args)...);
        }

        template <typename... Args>
        int update(const std::vector<T> &t, Args &&...args)
        {
            return sqlite.update(t, std::forward<Args>(args)...);
        }

        template <typename... Args>
        bool delete_records(Args &&...where_conditon)
        {
            return sqlite.delete_records<T>(std::forward<Args>(where_conditon)...);
        }
    };

} // namespace twc_database_msgs

#endif // _MESSAGE_OPTION_HPP_
