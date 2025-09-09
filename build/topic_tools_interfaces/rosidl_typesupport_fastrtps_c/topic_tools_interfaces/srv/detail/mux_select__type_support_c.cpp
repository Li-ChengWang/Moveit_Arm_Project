// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from topic_tools_interfaces:srv/MuxSelect.idl
// generated code does not contain a copyright notice
#include "topic_tools_interfaces/srv/detail/mux_select__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "topic_tools_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "topic_tools_interfaces/srv/detail/mux_select__struct.h"
#include "topic_tools_interfaces/srv/detail/mux_select__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // topic
#include "rosidl_runtime_c/string_functions.h"  // topic

// forward declare type support functions


using _MuxSelect_Request__ros_msg_type = topic_tools_interfaces__srv__MuxSelect_Request;

static bool _MuxSelect_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MuxSelect_Request__ros_msg_type * ros_message = static_cast<const _MuxSelect_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: topic
  {
    const rosidl_runtime_c__String * str = &ros_message->topic;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _MuxSelect_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MuxSelect_Request__ros_msg_type * ros_message = static_cast<_MuxSelect_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: topic
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->topic.data) {
      rosidl_runtime_c__String__init(&ros_message->topic);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->topic,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'topic'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_topic_tools_interfaces
size_t get_serialized_size_topic_tools_interfaces__srv__MuxSelect_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MuxSelect_Request__ros_msg_type * ros_message = static_cast<const _MuxSelect_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->topic.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _MuxSelect_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_topic_tools_interfaces__srv__MuxSelect_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_topic_tools_interfaces
size_t max_serialized_size_topic_tools_interfaces__srv__MuxSelect_Request(
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

  // member: topic
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
    using DataType = topic_tools_interfaces__srv__MuxSelect_Request;
    is_plain =
      (
      offsetof(DataType, topic) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MuxSelect_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_topic_tools_interfaces__srv__MuxSelect_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MuxSelect_Request = {
  "topic_tools_interfaces::srv",
  "MuxSelect_Request",
  _MuxSelect_Request__cdr_serialize,
  _MuxSelect_Request__cdr_deserialize,
  _MuxSelect_Request__get_serialized_size,
  _MuxSelect_Request__max_serialized_size
};

static rosidl_message_type_support_t _MuxSelect_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MuxSelect_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topic_tools_interfaces, srv, MuxSelect_Request)() {
  return &_MuxSelect_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "topic_tools_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "topic_tools_interfaces/srv/detail/mux_select__struct.h"
// already included above
// #include "topic_tools_interfaces/srv/detail/mux_select__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/string.h"  // prev_topic
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // prev_topic

// forward declare type support functions


using _MuxSelect_Response__ros_msg_type = topic_tools_interfaces__srv__MuxSelect_Response;

static bool _MuxSelect_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MuxSelect_Response__ros_msg_type * ros_message = static_cast<const _MuxSelect_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: prev_topic
  {
    const rosidl_runtime_c__String * str = &ros_message->prev_topic;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _MuxSelect_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MuxSelect_Response__ros_msg_type * ros_message = static_cast<_MuxSelect_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: prev_topic
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->prev_topic.data) {
      rosidl_runtime_c__String__init(&ros_message->prev_topic);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->prev_topic,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'prev_topic'\n");
      return false;
    }
  }

  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_topic_tools_interfaces
size_t get_serialized_size_topic_tools_interfaces__srv__MuxSelect_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MuxSelect_Response__ros_msg_type * ros_message = static_cast<const _MuxSelect_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name prev_topic
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->prev_topic.size + 1);
  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MuxSelect_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_topic_tools_interfaces__srv__MuxSelect_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_topic_tools_interfaces
size_t max_serialized_size_topic_tools_interfaces__srv__MuxSelect_Response(
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

  // member: prev_topic
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
  // member: success
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
    using DataType = topic_tools_interfaces__srv__MuxSelect_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MuxSelect_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_topic_tools_interfaces__srv__MuxSelect_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MuxSelect_Response = {
  "topic_tools_interfaces::srv",
  "MuxSelect_Response",
  _MuxSelect_Response__cdr_serialize,
  _MuxSelect_Response__cdr_deserialize,
  _MuxSelect_Response__get_serialized_size,
  _MuxSelect_Response__max_serialized_size
};

static rosidl_message_type_support_t _MuxSelect_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MuxSelect_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topic_tools_interfaces, srv, MuxSelect_Response)() {
  return &_MuxSelect_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "topic_tools_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "topic_tools_interfaces/srv/mux_select.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t MuxSelect__callbacks = {
  "topic_tools_interfaces::srv",
  "MuxSelect",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topic_tools_interfaces, srv, MuxSelect_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topic_tools_interfaces, srv, MuxSelect_Response)(),
};

static rosidl_service_type_support_t MuxSelect__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &MuxSelect__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topic_tools_interfaces, srv, MuxSelect)() {
  return &MuxSelect__handle;
}

#if defined(__cplusplus)
}
#endif
