// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topic_tools_interfaces:srv/MuxList.idl
// generated code does not contain a copyright notice

#ifndef TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__BUILDER_HPP_
#define TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topic_tools_interfaces/srv/detail/mux_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topic_tools_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::topic_tools_interfaces::srv::MuxList_Request>()
{
  return ::topic_tools_interfaces::srv::MuxList_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace topic_tools_interfaces


namespace topic_tools_interfaces
{

namespace srv
{

namespace builder
{

class Init_MuxList_Response_topics
{
public:
  Init_MuxList_Response_topics()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::topic_tools_interfaces::srv::MuxList_Response topics(::topic_tools_interfaces::srv::MuxList_Response::_topics_type arg)
  {
    msg_.topics = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topic_tools_interfaces::srv::MuxList_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::topic_tools_interfaces::srv::MuxList_Response>()
{
  return topic_tools_interfaces::srv::builder::Init_MuxList_Response_topics();
}

}  // namespace topic_tools_interfaces

#endif  // TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__BUILDER_HPP_
