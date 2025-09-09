// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from topic_tools_interfaces:srv/MuxSelect.idl
// generated code does not contain a copyright notice
#include "topic_tools_interfaces/srv/detail/mux_select__rosidl_typesupport_fastrtps_cpp.hpp"
#include "topic_tools_interfaces/srv/detail/mux_select__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace topic_tools_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topic_tools_interfaces
cdr_serialize(
  const topic_tools_interfaces::srv::MuxSelect_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: topic
  cdr << ros_message.topic;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topic_tools_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  topic_tools_interfaces::srv::MuxSelect_Request & ros_message)
{
  // Member: topic
  cdr >> ros_message.topic;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topic_tools_interfaces
get_serialized_size(
  const topic_tools_interfaces::srv::MuxSelect_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.topic.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topic_tools_interfaces
max_serialized_size_MuxSelect_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: topic
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = topic_tools_interfaces::srv::MuxSelect_Request;
    is_plain =
      (
      offsetof(DataType, topic) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MuxSelect_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const topic_tools_interfaces::srv::MuxSelect_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MuxSelect_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<topic_tools_interfaces::srv::MuxSelect_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MuxSelect_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const topic_tools_interfaces::srv::MuxSelect_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MuxSelect_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MuxSelect_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MuxSelect_Request__callbacks = {
  "topic_tools_interfaces::srv",
  "MuxSelect_Request",
  _MuxSelect_Request__cdr_serialize,
  _MuxSelect_Request__cdr_deserialize,
  _MuxSelect_Request__get_serialized_size,
  _MuxSelect_Request__max_serialized_size
};

static rosidl_message_type_support_t _MuxSelect_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MuxSelect_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace topic_tools_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_topic_tools_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<topic_tools_interfaces::srv::MuxSelect_Request>()
{
  return &topic_tools_interfaces::srv::typesupport_fastrtps_cpp::_MuxSelect_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, topic_tools_interfaces, srv, MuxSelect_Request)() {
  return &topic_tools_interfaces::srv::typesupport_fastrtps_cpp::_MuxSelect_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace topic_tools_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topic_tools_interfaces
cdr_serialize(
  const topic_tools_interfaces::srv::MuxSelect_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: prev_topic
  cdr << ros_message.prev_topic;
  // Member: success
  cdr << (ros_message.success ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topic_tools_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  topic_tools_interfaces::srv::MuxSelect_Response & ros_message)
{
  // Member: prev_topic
  cdr >> ros_message.prev_topic;

  // Member: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.success = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topic_tools_interfaces
get_serialized_size(
  const topic_tools_interfaces::srv::MuxSelect_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: prev_topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.prev_topic.size() + 1);
  // Member: success
  {
    size_t item_size = sizeof(ros_message.success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topic_tools_interfaces
max_serialized_size_MuxSelect_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: prev_topic
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = topic_tools_interfaces::srv::MuxSelect_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MuxSelect_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const topic_tools_interfaces::srv::MuxSelect_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MuxSelect_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<topic_tools_interfaces::srv::MuxSelect_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MuxSelect_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const topic_tools_interfaces::srv::MuxSelect_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MuxSelect_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MuxSelect_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MuxSelect_Response__callbacks = {
  "topic_tools_interfaces::srv",
  "MuxSelect_Response",
  _MuxSelect_Response__cdr_serialize,
  _MuxSelect_Response__cdr_deserialize,
  _MuxSelect_Response__get_serialized_size,
  _MuxSelect_Response__max_serialized_size
};

static rosidl_message_type_support_t _MuxSelect_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MuxSelect_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace topic_tools_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_topic_tools_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<topic_tools_interfaces::srv::MuxSelect_Response>()
{
  return &topic_tools_interfaces::srv::typesupport_fastrtps_cpp::_MuxSelect_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, topic_tools_interfaces, srv, MuxSelect_Response)() {
  return &topic_tools_interfaces::srv::typesupport_fastrtps_cpp::_MuxSelect_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace topic_tools_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _MuxSelect__callbacks = {
  "topic_tools_interfaces::srv",
  "MuxSelect",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, topic_tools_interfaces, srv, MuxSelect_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, topic_tools_interfaces, srv, MuxSelect_Response)(),
};

static rosidl_service_type_support_t _MuxSelect__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MuxSelect__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace topic_tools_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_topic_tools_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<topic_tools_interfaces::srv::MuxSelect>()
{
  return &topic_tools_interfaces::srv::typesupport_fastrtps_cpp::_MuxSelect__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, topic_tools_interfaces, srv, MuxSelect)() {
  return &topic_tools_interfaces::srv::typesupport_fastrtps_cpp::_MuxSelect__handle;
}

#ifdef __cplusplus
}
#endif
