// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from deadwheel_msgs:msg/DeadwheelTicks.idl
// generated code does not contain a copyright notice
#include "deadwheel_msgs/msg/detail/deadwheel_ticks__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
deadwheel_msgs__msg__DeadwheelTicks__init(deadwheel_msgs__msg__DeadwheelTicks * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    deadwheel_msgs__msg__DeadwheelTicks__fini(msg);
    return false;
  }
  // t0
  // t1
  // t2
  return true;
}

void
deadwheel_msgs__msg__DeadwheelTicks__fini(deadwheel_msgs__msg__DeadwheelTicks * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // t0
  // t1
  // t2
}

bool
deadwheel_msgs__msg__DeadwheelTicks__are_equal(const deadwheel_msgs__msg__DeadwheelTicks * lhs, const deadwheel_msgs__msg__DeadwheelTicks * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // t0
  if (lhs->t0 != rhs->t0) {
    return false;
  }
  // t1
  if (lhs->t1 != rhs->t1) {
    return false;
  }
  // t2
  if (lhs->t2 != rhs->t2) {
    return false;
  }
  return true;
}

bool
deadwheel_msgs__msg__DeadwheelTicks__copy(
  const deadwheel_msgs__msg__DeadwheelTicks * input,
  deadwheel_msgs__msg__DeadwheelTicks * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // t0
  output->t0 = input->t0;
  // t1
  output->t1 = input->t1;
  // t2
  output->t2 = input->t2;
  return true;
}

deadwheel_msgs__msg__DeadwheelTicks *
deadwheel_msgs__msg__DeadwheelTicks__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deadwheel_msgs__msg__DeadwheelTicks * msg = (deadwheel_msgs__msg__DeadwheelTicks *)allocator.allocate(sizeof(deadwheel_msgs__msg__DeadwheelTicks), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(deadwheel_msgs__msg__DeadwheelTicks));
  bool success = deadwheel_msgs__msg__DeadwheelTicks__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
deadwheel_msgs__msg__DeadwheelTicks__destroy(deadwheel_msgs__msg__DeadwheelTicks * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    deadwheel_msgs__msg__DeadwheelTicks__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
deadwheel_msgs__msg__DeadwheelTicks__Sequence__init(deadwheel_msgs__msg__DeadwheelTicks__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deadwheel_msgs__msg__DeadwheelTicks * data = NULL;

  if (size) {
    data = (deadwheel_msgs__msg__DeadwheelTicks *)allocator.zero_allocate(size, sizeof(deadwheel_msgs__msg__DeadwheelTicks), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = deadwheel_msgs__msg__DeadwheelTicks__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        deadwheel_msgs__msg__DeadwheelTicks__fini(&data[i - 1]);
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
deadwheel_msgs__msg__DeadwheelTicks__Sequence__fini(deadwheel_msgs__msg__DeadwheelTicks__Sequence * array)
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
      deadwheel_msgs__msg__DeadwheelTicks__fini(&array->data[i]);
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

deadwheel_msgs__msg__DeadwheelTicks__Sequence *
deadwheel_msgs__msg__DeadwheelTicks__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  deadwheel_msgs__msg__DeadwheelTicks__Sequence * array = (deadwheel_msgs__msg__DeadwheelTicks__Sequence *)allocator.allocate(sizeof(deadwheel_msgs__msg__DeadwheelTicks__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = deadwheel_msgs__msg__DeadwheelTicks__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
deadwheel_msgs__msg__DeadwheelTicks__Sequence__destroy(deadwheel_msgs__msg__DeadwheelTicks__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    deadwheel_msgs__msg__DeadwheelTicks__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
deadwheel_msgs__msg__DeadwheelTicks__Sequence__are_equal(const deadwheel_msgs__msg__DeadwheelTicks__Sequence * lhs, const deadwheel_msgs__msg__DeadwheelTicks__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!deadwheel_msgs__msg__DeadwheelTicks__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
deadwheel_msgs__msg__DeadwheelTicks__Sequence__copy(
  const deadwheel_msgs__msg__DeadwheelTicks__Sequence * input,
  deadwheel_msgs__msg__DeadwheelTicks__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(deadwheel_msgs__msg__DeadwheelTicks);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    deadwheel_msgs__msg__DeadwheelTicks * data =
      (deadwheel_msgs__msg__DeadwheelTicks *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!deadwheel_msgs__msg__DeadwheelTicks__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          deadwheel_msgs__msg__DeadwheelTicks__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!deadwheel_msgs__msg__DeadwheelTicks__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
