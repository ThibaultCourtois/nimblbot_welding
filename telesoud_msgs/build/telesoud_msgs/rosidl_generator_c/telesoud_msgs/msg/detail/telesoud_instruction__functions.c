// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from telesoud_msgs:msg/TelesoudInstruction.idl
// generated code does not contain a copyright notice
#include "telesoud_msgs/msg/detail/telesoud_instruction__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
telesoud_msgs__msg__TelesoudInstruction__init(telesoud_msgs__msg__TelesoudInstruction * msg)
{
  if (!msg) {
    return false;
  }
  // instruction_code
  // pose1
  // speed_vector
  // speed
  // free_drive_btn_status
  return true;
}

void
telesoud_msgs__msg__TelesoudInstruction__fini(telesoud_msgs__msg__TelesoudInstruction * msg)
{
  if (!msg) {
    return;
  }
  // instruction_code
  // pose1
  // speed_vector
  // speed
  // free_drive_btn_status
}

bool
telesoud_msgs__msg__TelesoudInstruction__are_equal(const telesoud_msgs__msg__TelesoudInstruction * lhs, const telesoud_msgs__msg__TelesoudInstruction * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // instruction_code
  if (lhs->instruction_code != rhs->instruction_code) {
    return false;
  }
  // pose1
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->pose1[i] != rhs->pose1[i]) {
      return false;
    }
  }
  // speed_vector
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->speed_vector[i] != rhs->speed_vector[i]) {
      return false;
    }
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // free_drive_btn_status
  if (lhs->free_drive_btn_status != rhs->free_drive_btn_status) {
    return false;
  }
  return true;
}

bool
telesoud_msgs__msg__TelesoudInstruction__copy(
  const telesoud_msgs__msg__TelesoudInstruction * input,
  telesoud_msgs__msg__TelesoudInstruction * output)
{
  if (!input || !output) {
    return false;
  }
  // instruction_code
  output->instruction_code = input->instruction_code;
  // pose1
  for (size_t i = 0; i < 6; ++i) {
    output->pose1[i] = input->pose1[i];
  }
  // speed_vector
  for (size_t i = 0; i < 6; ++i) {
    output->speed_vector[i] = input->speed_vector[i];
  }
  // speed
  output->speed = input->speed;
  // free_drive_btn_status
  output->free_drive_btn_status = input->free_drive_btn_status;
  return true;
}

telesoud_msgs__msg__TelesoudInstruction *
telesoud_msgs__msg__TelesoudInstruction__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  telesoud_msgs__msg__TelesoudInstruction * msg = (telesoud_msgs__msg__TelesoudInstruction *)allocator.allocate(sizeof(telesoud_msgs__msg__TelesoudInstruction), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(telesoud_msgs__msg__TelesoudInstruction));
  bool success = telesoud_msgs__msg__TelesoudInstruction__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
telesoud_msgs__msg__TelesoudInstruction__destroy(telesoud_msgs__msg__TelesoudInstruction * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    telesoud_msgs__msg__TelesoudInstruction__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
telesoud_msgs__msg__TelesoudInstruction__Sequence__init(telesoud_msgs__msg__TelesoudInstruction__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  telesoud_msgs__msg__TelesoudInstruction * data = NULL;

  if (size) {
    data = (telesoud_msgs__msg__TelesoudInstruction *)allocator.zero_allocate(size, sizeof(telesoud_msgs__msg__TelesoudInstruction), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = telesoud_msgs__msg__TelesoudInstruction__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        telesoud_msgs__msg__TelesoudInstruction__fini(&data[i - 1]);
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
telesoud_msgs__msg__TelesoudInstruction__Sequence__fini(telesoud_msgs__msg__TelesoudInstruction__Sequence * array)
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
      telesoud_msgs__msg__TelesoudInstruction__fini(&array->data[i]);
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

telesoud_msgs__msg__TelesoudInstruction__Sequence *
telesoud_msgs__msg__TelesoudInstruction__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  telesoud_msgs__msg__TelesoudInstruction__Sequence * array = (telesoud_msgs__msg__TelesoudInstruction__Sequence *)allocator.allocate(sizeof(telesoud_msgs__msg__TelesoudInstruction__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = telesoud_msgs__msg__TelesoudInstruction__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
telesoud_msgs__msg__TelesoudInstruction__Sequence__destroy(telesoud_msgs__msg__TelesoudInstruction__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    telesoud_msgs__msg__TelesoudInstruction__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
telesoud_msgs__msg__TelesoudInstruction__Sequence__are_equal(const telesoud_msgs__msg__TelesoudInstruction__Sequence * lhs, const telesoud_msgs__msg__TelesoudInstruction__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!telesoud_msgs__msg__TelesoudInstruction__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
telesoud_msgs__msg__TelesoudInstruction__Sequence__copy(
  const telesoud_msgs__msg__TelesoudInstruction__Sequence * input,
  telesoud_msgs__msg__TelesoudInstruction__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(telesoud_msgs__msg__TelesoudInstruction);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    telesoud_msgs__msg__TelesoudInstruction * data =
      (telesoud_msgs__msg__TelesoudInstruction *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!telesoud_msgs__msg__TelesoudInstruction__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          telesoud_msgs__msg__TelesoudInstruction__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!telesoud_msgs__msg__TelesoudInstruction__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
