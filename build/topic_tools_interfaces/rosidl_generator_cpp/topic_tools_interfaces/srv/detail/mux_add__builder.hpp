// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topic_tools_interfaces:srv/MuxAdd.idl
// generated code does not contain a copyright notice

#ifndef TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_ADD__BUILDER_HPP_
#define TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_ADD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topic_tools_interfaces/srv/detail/mux_add__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topic_tools_interfaces
{

namespace srv
{

namespace builder
{

class Init_MuxAdd_Request_topic
{
public:
  Init_MuxAdd_Request_topic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::topic_tools_interfaces::srv::MuxAdd_Request topic(::topic_tools_interfaces::srv::MuxAdd_Request::_topic_type arg)
  {
    msg_.topic = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topic_tools_interfaces::srv::MuxAdd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::topic_tools_interfaces::srv::MuxAdd_Request>()
{
  return topic_tools_interfaces::srv::builder::Init_MuxAdd_Request_topic();
}

}  // namespace topic_tools_interfaces


namespace topic_tools_interfaces
{

namespace srv
{

namespace builder
{

class Init_MuxAdd_Response_success
{
public:
  Init_MuxAdd_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::topic_tools_interfaces::srv::MuxAdd_Response success(::topic_tools_interfaces::srv::MuxAdd_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topic_tools_interfaces::srv::MuxAdd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::topic_tools_interfaces::srv::MuxAdd_Response>()
{
  return topic_tools_interfaces::srv::builder::Init_MuxAdd_Response_success();
}

}  // namespace topic_tools_interfaces

#endif  // TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_ADD__BUILDER_HPP_
