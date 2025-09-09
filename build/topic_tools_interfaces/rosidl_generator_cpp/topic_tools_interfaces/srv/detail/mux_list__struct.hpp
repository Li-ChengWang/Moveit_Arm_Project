// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from topic_tools_interfaces:srv/MuxList.idl
// generated code does not contain a copyright notice

#ifndef TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__STRUCT_HPP_
#define TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__topic_tools_interfaces__srv__MuxList_Request __attribute__((deprecated))
#else
# define DEPRECATED__topic_tools_interfaces__srv__MuxList_Request __declspec(deprecated)
#endif

namespace topic_tools_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MuxList_Request_
{
  using Type = MuxList_Request_<ContainerAllocator>;

  explicit MuxList_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit MuxList_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topic_tools_interfaces__srv__MuxList_Request
    std::shared_ptr<topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topic_tools_interfaces__srv__MuxList_Request
    std::shared_ptr<topic_tools_interfaces::srv::MuxList_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MuxList_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const MuxList_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MuxList_Request_

// alias to use template instance with default allocator
using MuxList_Request =
  topic_tools_interfaces::srv::MuxList_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace topic_tools_interfaces


#ifndef _WIN32
# define DEPRECATED__topic_tools_interfaces__srv__MuxList_Response __attribute__((deprecated))
#else
# define DEPRECATED__topic_tools_interfaces__srv__MuxList_Response __declspec(deprecated)
#endif

namespace topic_tools_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MuxList_Response_
{
  using Type = MuxList_Response_<ContainerAllocator>;

  explicit MuxList_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MuxList_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _topics_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _topics_type topics;

  // setters for named parameter idiom
  Type & set__topics(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->topics = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topic_tools_interfaces__srv__MuxList_Response
    std::shared_ptr<topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topic_tools_interfaces__srv__MuxList_Response
    std::shared_ptr<topic_tools_interfaces::srv::MuxList_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MuxList_Response_ & other) const
  {
    if (this->topics != other.topics) {
      return false;
    }
    return true;
  }
  bool operator!=(const MuxList_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MuxList_Response_

// alias to use template instance with default allocator
using MuxList_Response =
  topic_tools_interfaces::srv::MuxList_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace topic_tools_interfaces

namespace topic_tools_interfaces
{

namespace srv
{

struct MuxList
{
  using Request = topic_tools_interfaces::srv::MuxList_Request;
  using Response = topic_tools_interfaces::srv::MuxList_Response;
};

}  // namespace srv

}  // namespace topic_tools_interfaces

#endif  // TOPIC_TOOLS_INTERFACES__SRV__DETAIL__MUX_LIST__STRUCT_HPP_
