// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from topic_tools_interfaces:srv/MuxDelete.idl
// generated code does not contain a copyright notice

#ifndef TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_DELETE__TRAITS_HPP_
#define TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_DELETE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "topic_tools_interfaces/srv/detail/mux_delete__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace topic_tools_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MuxDelete_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic
  {
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MuxDelete_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MuxDelete_Request & msg, bool use_flow_style = false)
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
  const topic_tools_interfaces::srv::MuxDelete_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  topic_tools_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topic_tools_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const topic_tools_interfaces::srv::MuxDelete_Request & msg)
{
  return topic_tools_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<topic_tools_interfaces::srv::MuxDelete_Request>()
{
  return "topic_tools_interfaces::srv::MuxDelete_Request";
}

template<>
inline const char * name<topic_tools_interfaces::srv::MuxDelete_Request>()
{
  return "topic_tools_interfaces/srv/MuxDelete_Request";
}

template<>
struct has_fixed_size<topic_tools_interfaces::srv::MuxDelete_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topic_tools_interfaces::srv::MuxDelete_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topic_tools_interfaces::srv::MuxDelete_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace topic_tools_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MuxDelete_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MuxDelete_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MuxDelete_Response & msg, bool use_flow_style = false)
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
  const topic_tools_interfaces::srv::MuxDelete_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  topic_tools_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topic_tools_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const topic_tools_interfaces::srv::MuxDelete_Response & msg)
{
  return topic_tools_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<topic_tools_interfaces::srv::MuxDelete_Response>()
{
  return "topic_tools_interfaces::srv::MuxDelete_Response";
}

template<>
inline const char * name<topic_tools_interfaces::srv::MuxDelete_Response>()
{
  return "topic_tools_interfaces/srv/MuxDelete_Response";
}

template<>
struct has_fixed_size<topic_tools_interfaces::srv::MuxDelete_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<topic_tools_interfaces::srv::MuxDelete_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<topic_tools_interfaces::srv::MuxDelete_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<topic_tools_interfaces::srv::MuxDelete>()
{
  return "topic_tools_interfaces::srv::MuxDelete";
}

template<>
inline const char * name<topic_tools_interfaces::srv::MuxDelete>()
{
  return "topic_tools_interfaces/srv/MuxDelete";
}

template<>
struct has_fixed_size<topic_tools_interfaces::srv::MuxDelete>
  : std::integral_constant<
    bool,
    has_fixed_size<topic_tools_interfaces::srv::MuxDelete_Request>::value &&
    has_fixed_size<topic_tools_interfaces::srv::MuxDelete_Response>::value
  >
{
};

template<>
struct has_bounded_size<topic_tools_interfaces::srv::MuxDelete>
  : std::integral_constant<
    bool,
    has_bounded_size<topic_tools_interfaces::srv::MuxDelete_Request>::value &&
    has_bounded_size<topic_tools_interfaces::srv::MuxDelete_Response>::value
  >
{
};

template<>
struct is_service<topic_tools_interfaces::srv::MuxDelete>
  : std::true_type
{
};

template<>
struct is_service_request<topic_tools_interfaces::srv::MuxDelete_Request>
  : std::true_type
{
};

template<>
struct is_service_response<topic_tools_interfaces::srv::MuxDelete_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_DELETE__TRAITS_HPP_
