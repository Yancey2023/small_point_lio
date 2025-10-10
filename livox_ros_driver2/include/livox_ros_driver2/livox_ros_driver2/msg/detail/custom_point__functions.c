// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice
#include "livox_ros_driver2/msg/detail/custom_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
livox_ros_driver2__msg__CustomPoint__init(livox_ros_driver2__msg__CustomPoint * msg)
{
  if (!msg) {
    return false;
  }
  // offset_time
  // x
  // y
  // z
  // reflectivity
  // tag
  // line
  return true;
}

void
livox_ros_driver2__msg__CustomPoint__fini(livox_ros_driver2__msg__CustomPoint * msg)
{
  if (!msg) {
    return;
  }
  // offset_time
  // x
  // y
  // z
  // reflectivity
  // tag
  // line
}

bool
livox_ros_driver2__msg__CustomPoint__are_equal(const livox_ros_driver2__msg__CustomPoint * lhs, const livox_ros_driver2__msg__CustomPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // offset_time
  if (lhs->offset_time != rhs->offset_time) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // reflectivity
  if (lhs->reflectivity != rhs->reflectivity) {
    return false;
  }
  // tag
  if (lhs->tag != rhs->tag) {
    return false;
  }
  // line
  if (lhs->line != rhs->line) {
    return false;
  }
  return true;
}

bool
livox_ros_driver2__msg__CustomPoint__copy(
  const livox_ros_driver2__msg__CustomPoint * input,
  livox_ros_driver2__msg__CustomPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // offset_time
  output->offset_time = input->offset_time;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // reflectivity
  output->reflectivity = input->reflectivity;
  // tag
  output->tag = input->tag;
  // line
  output->line = input->line;
  return true;
}

livox_ros_driver2__msg__CustomPoint *
livox_ros_driver2__msg__CustomPoint__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  livox_ros_driver2__msg__CustomPoint * msg = (livox_ros_driver2__msg__CustomPoint *)allocator.allocate(sizeof(livox_ros_driver2__msg__CustomPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(livox_ros_driver2__msg__CustomPoint));
  bool success = livox_ros_driver2__msg__CustomPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
livox_ros_driver2__msg__CustomPoint__destroy(livox_ros_driver2__msg__CustomPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    livox_ros_driver2__msg__CustomPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
livox_ros_driver2__msg__CustomPoint__Sequence__init(livox_ros_driver2__msg__CustomPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  livox_ros_driver2__msg__CustomPoint * data = NULL;

  if (size) {
    data = (livox_ros_driver2__msg__CustomPoint *)allocator.zero_allocate(size, sizeof(livox_ros_driver2__msg__CustomPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = livox_ros_driver2__msg__CustomPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        livox_ros_driver2__msg__CustomPoint__fini(&data[i - 1]);
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
livox_ros_driver2__msg__CustomPoint__Sequence__fini(livox_ros_driver2__msg__CustomPoint__Sequence * array)
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
      livox_ros_driver2__msg__CustomPoint__fini(&array->data[i]);
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

livox_ros_driver2__msg__CustomPoint__Sequence *
livox_ros_driver2__msg__CustomPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  livox_ros_driver2__msg__CustomPoint__Sequence * array = (livox_ros_driver2__msg__CustomPoint__Sequence *)allocator.allocate(sizeof(livox_ros_driver2__msg__CustomPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = livox_ros_driver2__msg__CustomPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
livox_ros_driver2__msg__CustomPoint__Sequence__destroy(livox_ros_driver2__msg__CustomPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    livox_ros_driver2__msg__CustomPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
livox_ros_driver2__msg__CustomPoint__Sequence__are_equal(const livox_ros_driver2__msg__CustomPoint__Sequence * lhs, const livox_ros_driver2__msg__CustomPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!livox_ros_driver2__msg__CustomPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
livox_ros_driver2__msg__CustomPoint__Sequence__copy(
  const livox_ros_driver2__msg__CustomPoint__Sequence * input,
  livox_ros_driver2__msg__CustomPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(livox_ros_driver2__msg__CustomPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    livox_ros_driver2__msg__CustomPoint * data =
      (livox_ros_driver2__msg__CustomPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!livox_ros_driver2__msg__CustomPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          livox_ros_driver2__msg__CustomPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!livox_ros_driver2__msg__CustomPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
