// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topic_tools_interfaces:srv/MuxSelect.idl
// generated code does not contain a copyright notice

#ifndef TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_SELECT__BUILDER_HPP_
#define TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_SELECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topic_tools_interfaces/srv/detail/mux_select__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topic_tools_interfaces
{

namespace srv
{

namespace builder
{

class Init_MuxSelect_Request_topic
{
public:
  Init_MuxSelect_Request_topic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::topic_tools_interfaces::srv::MuxSelect_Request topic(::topic_tools_interfaces::srv::MuxSelect_Request::_topic_type arg)
  {
    msg_.topic = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topic_tools_interfaces::srv::MuxSelect_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::topic_tools_interfaces::srv::MuxSelect_Request>()
{
  return topic_tools_interfaces::srv::builder::Init_MuxSelect_Request_topic();
}

}  // namespace topic_tools_interfaces


namespace topic_tools_interfaces
{

namespace srv
{

namespace builder
{

class Init_MuxSelect_Response_success
{
public:
  explicit Init_MuxSelect_Response_success(::topic_tools_interfaces::srv::MuxSelect_Response & msg)
  : msg_(msg)
  {}
  ::topic_tools_interfaces::srv::MuxSelect_Response success(::topic_tools_interfaces::srv::MuxSelect_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topic_tools_interfaces::srv::MuxSelect_Response msg_;
};

class Init_MuxSelect_Response_prev_topic
{
public:
  Init_MuxSelect_Response_prev_topic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MuxSelect_Response_success prev_topic(::topic_tools_interfaces::srv::MuxSelect_Response::_prev_topic_type arg)
  {
    msg_.prev_topic = std::move(arg);
    return Init_MuxSelect_Response_success(msg_);
  }

private:
  ::topic_tools_interfaces::srv::MuxSelect_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::topic_tools_interfaces::srv::MuxSelect_Response>()
{
  return topic_tools_interfaces::srv::builder::Init_MuxSelect_Response_prev_topic();
}

}  // namespace topic_tools_interfaces

#endif  // TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_SELECT__BUILDER_HPP_
