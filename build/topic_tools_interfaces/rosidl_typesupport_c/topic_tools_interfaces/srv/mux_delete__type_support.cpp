// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from topic_tools_interfaces:srv/MuxDelete.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "topic_tools_interfaces/srv/detail/mux_delete__struct.h"
#include "topic_tools_interfaces/srv/detail/mux_delete__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace topic_tools_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _MuxDelete_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MuxDelete_Request_type_support_ids_t;

static const _MuxDelete_Request_type_support_ids_t _MuxDelete_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MuxDelete_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MuxDelete_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MuxDelete_Request_type_support_symbol_names_t _MuxDelete_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topic_tools_interfaces, srv, MuxDelete_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topic_tools_interfaces, srv, MuxDelete_Request)),
  }
};

typedef struct _MuxDelete_Request_type_support_data_t
{
  void * data[2];
} _MuxDelete_Request_type_support_data_t;

static _MuxDelete_Request_type_support_data_t _MuxDelete_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MuxDelete_Request_message_typesupport_map = {
  2,
  "topic_tools_interfaces",
  &_MuxDelete_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MuxDelete_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MuxDelete_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MuxDelete_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MuxDelete_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace topic_tools_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, topic_tools_interfaces, srv, MuxDelete_Request)() {
  return &::topic_tools_interfaces::srv::rosidl_typesupport_c::MuxDelete_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "topic_tools_interfaces/srv/detail/mux_delete__struct.h"
// already included above
// #include "topic_tools_interfaces/srv/detail/mux_delete__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace topic_tools_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _MuxDelete_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MuxDelete_Response_type_support_ids_t;

static const _MuxDelete_Response_type_support_ids_t _MuxDelete_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MuxDelete_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MuxDelete_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MuxDelete_Response_type_support_symbol_names_t _MuxDelete_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topic_tools_interfaces, srv, MuxDelete_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topic_tools_interfaces, srv, MuxDelete_Response)),
  }
};

typedef struct _MuxDelete_Response_type_support_data_t
{
  void * data[2];
} _MuxDelete_Response_type_support_data_t;

static _MuxDelete_Response_type_support_data_t _MuxDelete_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MuxDelete_Response_message_typesupport_map = {
  2,
  "topic_tools_interfaces",
  &_MuxDelete_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MuxDelete_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MuxDelete_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MuxDelete_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MuxDelete_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace topic_tools_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, topic_tools_interfaces, srv, MuxDelete_Response)() {
  return &::topic_tools_interfaces::srv::rosidl_typesupport_c::MuxDelete_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "topic_tools_interfaces/srv/detail/mux_delete__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace topic_tools_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _MuxDelete_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MuxDelete_type_support_ids_t;

static const _MuxDelete_type_support_ids_t _MuxDelete_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MuxDelete_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MuxDelete_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MuxDelete_type_support_symbol_names_t _MuxDelete_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topic_tools_interfaces, srv, MuxDelete)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topic_tools_interfaces, srv, MuxDelete)),
  }
};

typedef struct _MuxDelete_type_support_data_t
{
  void * data[2];
} _MuxDelete_type_support_data_t;

static _MuxDelete_type_support_data_t _MuxDelete_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MuxDelete_service_typesupport_map = {
  2,
  "topic_tools_interfaces",
  &_MuxDelete_service_typesupport_ids.typesupport_identifier[0],
  &_MuxDelete_service_typesupport_symbol_names.symbol_name[0],
  &_MuxDelete_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MuxDelete_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MuxDelete_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace topic_tools_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, topic_tools_interfaces, srv, MuxDelete)() {
  return &::topic_tools_interfaces::srv::rosidl_typesupport_c::MuxDelete_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
