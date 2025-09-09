// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from topic_tools_interfaces:srv/MuxList.idl
// generated code does not contain a copyright notice
#include "topic_tools_interfaces/srv/detail/mux_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
topic_tools_interfaces__srv__MuxList_Request__init(topic_tools_interfaces__srv__MuxList_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
topic_tools_interfaces__srv__MuxList_Request__fini(topic_tools_interfaces__srv__MuxList_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
topic_tools_interfaces__srv__MuxList_Request__are_equal(const topic_tools_interfaces__srv__MuxList_Request * lhs, const topic_tools_interfaces__srv__MuxList_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
topic_tools_interfaces__srv__MuxList_Request__copy(
  const topic_tools_interfaces__srv__MuxList_Request * input,
  topic_tools_interfaces__srv__MuxList_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

topic_tools_interfaces__srv__MuxList_Request *
topic_tools_interfaces__srv__MuxList_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topic_tools_interfaces__srv__MuxList_Request * msg = (topic_tools_interfaces__srv__MuxList_Request *)allocator.allocate(sizeof(topic_tools_interfaces__srv__MuxList_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(topic_tools_interfaces__srv__MuxList_Request));
  bool success = topic_tools_interfaces__srv__MuxList_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
topic_tools_interfaces__srv__MuxList_Request__destroy(topic_tools_interfaces__srv__MuxList_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    topic_tools_interfaces__srv__MuxList_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
topic_tools_interfaces__srv__MuxList_Request__Sequence__init(topic_tools_interfaces__srv__MuxList_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topic_tools_interfaces__srv__MuxList_Request * data = NULL;

  if (size) {
    data = (topic_tools_interfaces__srv__MuxList_Request *)allocator.zero_allocate(size, sizeof(topic_tools_interfaces__srv__MuxList_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = topic_tools_interfaces__srv__MuxList_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        topic_tools_interfaces__srv__MuxList_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
topic_tools_interfaces__srv__MuxList_Request__Sequence__fini(topic_tools_interfaces__srv__MuxList_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      topic_tools_interfaces__srv__MuxList_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

topic_tools_interfaces__srv__MuxList_Request__Sequence *
topic_tools_interfaces__srv__MuxList_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topic_tools_interfaces__srv__MuxList_Request__Sequence * array = (topic_tools_interfaces__srv__MuxList_Request__Sequence *)allocator.allocate(sizeof(topic_tools_interfaces__srv__MuxList_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = topic_tools_interfaces__srv__MuxList_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
topic_tools_interfaces__srv__MuxList_Request__Sequence__destroy(topic_tools_interfaces__srv__MuxList_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    topic_tools_interfaces__srv__MuxList_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
topic_tools_interfaces__srv__MuxList_Request__Sequence__are_equal(const topic_tools_interfaces__srv__MuxList_Request__Sequence * lhs, const topic_tools_interfaces__srv__MuxList_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!topic_tools_interfaces__srv__MuxList_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
topic_tools_interfaces__srv__MuxList_Request__Sequence__copy(
  const topic_tools_interfaces__srv__MuxList_Request__Sequence * input,
  topic_tools_interfaces__srv__MuxList_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(topic_tools_interfaces__srv__MuxList_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    topic_tools_interfaces__srv__MuxList_Request * data =
      (topic_tools_interfaces__srv__MuxList_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!topic_tools_interfaces__srv__MuxList_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          topic_tools_interfaces__srv__MuxList_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!topic_tools_interfaces__srv__MuxList_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `topics`
#include "rosidl_runtime_c/string_functions.h"

bool
topic_tools_interfaces__srv__MuxList_Response__init(topic_tools_interfaces__srv__MuxList_Response * msg)
{
  if (!msg) {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__init(&msg->topics, 0)) {
    topic_tools_interfaces__srv__MuxList_Response__fini(msg);
    return false;
  }
  return true;
}

void
topic_tools_interfaces__srv__MuxList_Response__fini(topic_tools_interfaces__srv__MuxList_Response * msg)
{
  if (!msg) {
    return;
  }
  // topics
  rosidl_runtime_c__String__Sequence__fini(&msg->topics);
}

bool
topic_tools_interfaces__srv__MuxList_Response__are_equal(const topic_tools_interfaces__srv__MuxList_Response * lhs, const topic_tools_interfaces__srv__MuxList_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->topics), &(rhs->topics)))
  {
    return false;
  }
  return true;
}

bool
topic_tools_interfaces__srv__MuxList_Response__copy(
  const topic_tools_interfaces__srv__MuxList_Response * input,
  topic_tools_interfaces__srv__MuxList_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->topics), &(output->topics)))
  {
    return false;
  }
  return true;
}

topic_tools_interfaces__srv__MuxList_Response *
topic_tools_interfaces__srv__MuxList_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topic_tools_interfaces__srv__MuxList_Response * msg = (topic_tools_interfaces__srv__MuxList_Response *)allocator.allocate(sizeof(topic_tools_interfaces__srv__MuxList_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(topic_tools_interfaces__srv__MuxList_Response));
  bool success = topic_tools_interfaces__srv__MuxList_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
topic_tools_interfaces__srv__MuxList_Response__destroy(topic_tools_interfaces__srv__MuxList_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    topic_tools_interfaces__srv__MuxList_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
topic_tools_interfaces__srv__MuxList_Response__Sequence__init(topic_tools_interfaces__srv__MuxList_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topic_tools_interfaces__srv__MuxList_Response * data = NULL;

  if (size) {
    data = (topic_tools_interfaces__srv__MuxList_Response *)allocator.zero_allocate(size, sizeof(topic_tools_interfaces__srv__MuxList_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = topic_tools_interfaces__srv__MuxList_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        topic_tools_interfaces__srv__MuxList_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
topic_tools_interfaces__srv__MuxList_Response__Sequence__fini(topic_tools_interfaces__srv__MuxList_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      topic_tools_interfaces__srv__MuxList_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

topic_tools_interfaces__srv__MuxList_Response__Sequence *
topic_tools_interfaces__srv__MuxList_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topic_tools_interfaces__srv__MuxList_Response__Sequence * array = (topic_tools_interfaces__srv__MuxList_Response__Sequence *)allocator.allocate(sizeof(topic_tools_interfaces__srv__MuxList_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = topic_tools_interfaces__srv__MuxList_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
topic_tools_interfaces__srv__MuxList_Response__Sequence__destroy(topic_tools_interfaces__srv__MuxList_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    topic_tools_interfaces__srv__MuxList_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
topic_tools_interfaces__srv__MuxList_Response__Sequence__are_equal(const topic_tools_interfaces__srv__MuxList_Response__Sequence * lhs, const topic_tools_interfaces__srv__MuxList_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!topic_tools_interfaces__srv__MuxList_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
topic_tools_interfaces__srv__MuxList_Response__Sequence__copy(
  const topic_tools_interfaces__srv__MuxList_Response__Sequence * input,
  topic_tools_interfaces__srv__MuxList_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(topic_tools_interfaces__srv__MuxList_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    topic_tools_interfaces__srv__MuxList_Response * data =
      (topic_tools_interfaces__srv__MuxList_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!topic_tools_interfaces__srv__MuxList_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          topic_tools_interfaces__srv__MuxList_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!topic_tools_interfaces__srv__MuxList_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
