// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from topic_tools_interfaces:srv/MuxAdd.idl
// generated code does not contain a copyright notice

#ifndef TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_ADD__STRUCT_H_
#define TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_ADD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'topic'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MuxAdd in the package topic_tools_interfaces.
typedef struct topic_tools_interfaces__srv__MuxAdd_Request
{
  rosidl_runtime_c__String topic;
} topic_tools_interfaces__srv__MuxAdd_Request;

// Struct for a sequence of topic_tools_interfaces__srv__MuxAdd_Request.
typedef struct topic_tools_interfaces__srv__MuxAdd_Request__Sequence
{
  topic_tools_interfaces__srv__MuxAdd_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topic_tools_interfaces__srv__MuxAdd_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MuxAdd in the package topic_tools_interfaces.
typedef struct topic_tools_interfaces__srv__MuxAdd_Response
{
  bool success;
} topic_tools_interfaces__srv__MuxAdd_Response;

// Struct for a sequence of topic_tools_interfaces__srv__MuxAdd_Response.
typedef struct topic_tools_interfaces__srv__MuxAdd_Response__Sequence
{
  topic_tools_interfaces__srv__MuxAdd_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topic_tools_interfaces__srv__MuxAdd_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_ADD__STRUCT_H_
