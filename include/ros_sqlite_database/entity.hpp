#ifndef _SQLITE_WAPPER_ENTITY_HPP_
#define _SQLITE_WAPPER__ENTITY_HPP_

#include <set>
#include <string>

struct sqlite_not_null_t
{
    std::set<std::string> fields;
};

struct sqlite_key_t
{
    std::string fields;
};

struct sqlite_auto_key_t
{
    std::string fields;
};

struct sqlite_unique_t
{
    std::string fields;
};

#endif //_SQLITE_WAPPER_ENTITY_HPP_
