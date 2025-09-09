// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from topic_tools_interfaces:srv/MuxList.idl
// generated code does not contain a copyright notice

#ifndef TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__TRAITS_HPP_
#define TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "topic_tools_interfaces/srv/detail/mux_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace topic_tools_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MuxList_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MuxList_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MuxList_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace topic_tools_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use topic_tools_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const topic_tools_interfaces::srv::MuxList_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  topic_tools_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topic_tools_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const topic_tools_interfaces::srv::MuxList_Request & msg)
{
  return topic_tools_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<topic_tools_interfaces::srv::MuxList_Request>()
{
  return "topic_tools_interfaces::srv::MuxList_Request";
}

template<>
inline const char * name<topic_tools_interfaces::srv::MuxList_Request>()
{
  return "topic_tools_interfaces/srv/MuxList_Request";
}

template<>
struct has_fixed_size<topic_tools_interfaces::srv::MuxList_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<topic_tools_interfaces::srv::MuxList_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<topic_tools_interfaces::srv::MuxList_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace topic_tools_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MuxList_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: topics
  {
    if (msg.topics.size() == 0) {
      out << "topics: []";
    } else {
      out << "topics: [";
      size_t pending_items = msg.topics.size();
      for (auto item : msg.topics) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MuxList_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.topics.size() == 0) {
      out << "topics: []\n";
    } else {
      out << "topics:\n";
      for (auto item : msg.topics) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MuxList_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace topic_tools_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use topic_tools_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const topic_tools_interfaces::srv::MuxList_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  topic_tools_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topic_tools_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const topic_tools_interfaces::srv::MuxList_Response & msg)
{
  return topic_tools_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<topic_tools_interfaces::srv::MuxList_Response>()
{
  return "topic_tools_interfaces::srv::MuxList_Response";
}

template<>
inline const char * name<topic_tools_interfaces::srv::MuxList_Response>()
{
  return "topic_tools_interfaces/srv/MuxList_Response";
}

template<>
struct has_fixed_size<topic_tools_interfaces::srv::MuxList_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topic_tools_interfaces::srv::MuxList_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topic_tools_interfaces::srv::MuxList_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<topic_tools_interfaces::srv::MuxList>()
{
  return "topic_tools_interfaces::srv::MuxList";
}

template<>
inline const char * name<topic_tools_interfaces::srv::MuxList>()
{
  return "topic_tools_interfaces/srv/MuxList";
}

template<>
struct has_fixed_size<topic_tools_interfaces::srv::MuxList>
  : std::integral_constant<
    bool,
    has_fixed_size<topic_tools_interfaces::srv::MuxList_Request>::value &&
    has_fixed_size<topic_tools_interfaces::srv::MuxList_Response>::value
  >
{
};

template<>
struct has_bounded_size<topic_tools_interfaces::srv::MuxList>
  : std::integral_constant<
    bool,
    has_bounded_size<topic_tools_interfaces::srv::MuxList_Request>::value &&
    has_bounded_size<topic_tools_interfaces::srv::MuxList_Response>::value
  >
{
};

template<>
struct is_service<topic_tools_interfaces::srv::MuxList>
  : std::true_type
{
};

template<>
struct is_service_request<topic_tools_interfaces::srv::MuxList_Request>
  : std::true_type
{
};

template<>
struct is_service_response<topic_tools_interfaces::srv::MuxList_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__TRAITS_HPP_
