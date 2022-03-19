#include <sqlite3.h>
#include <string>
#include <string_view>
#include <array>

using namespace std::string_view_literals;

#ifndef _SQLITE_WAPPER_TYPE_MAPPING_HPP_
#define _SQLITE_WAPPER_TYPE_MAPPING_HPP_

namespace sqlite_wapper
{
	template <class T>
	struct identity
	{
	};

#define REGISTER_TYPE(Type, Index)                                                        \
	inline constexpr int type_to_id(identity<Type>) noexcept { return Index; }            \
	inline constexpr auto id_to_type(std::integral_constant<std::size_t, Index>) noexcept \
	{                                                                                     \
		Type res{};                                                                       \
		return res;                                                                       \
	}

	namespace sqlite_type_remmap
	{
		REGISTER_TYPE(int, SQLITE_INTEGER)
		REGISTER_TYPE(double, SQLITE_FLOAT)

		inline int type_to_id(identity<std::string>) noexcept { return SQLITE_TEXT; }
		inline std::string id_to_type(std::integral_constant<std::size_t, SQLITE_TEXT>) noexcept
		{
			std::string res{};
			return res;
		}

		inline constexpr auto type_to_name(identity<bool>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<char>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<unsigned char>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<short>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<unsigned short>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<int>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<unsigned int>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<long int>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<unsigned long int>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<long long int>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<unsigned long long int>) noexcept { return "INTEGER"sv; }
		inline constexpr auto type_to_name(identity<float>) noexcept { return "FLOAT"sv; }
		inline constexpr auto type_to_name(identity<double>) noexcept { return "DOUBLE"sv; }
		inline constexpr auto type_to_name(identity<std::vector<char>>) noexcept { return "BLOB"sv; }
		inline constexpr auto type_to_name(identity<std::vector<unsigned char>>) noexcept { return "BLOB"sv; }

		inline auto type_to_name(identity<std::string>) noexcept { return "TEXT"sv; }
		template <size_t N>
		inline constexpr auto type_to_name(identity<std::array<char, N>>) noexcept
		{
			std::string s = "varchar(" + std::to_string(N) + ")";
			return s;
		}
	}
}

#endif // _SQLITE_WAPPER_TYPE_MAPPING_HPP_
