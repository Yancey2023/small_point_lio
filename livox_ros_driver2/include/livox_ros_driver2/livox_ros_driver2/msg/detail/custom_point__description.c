// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice

#include "livox_ros_driver2/msg/detail/custom_point__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
const rosidl_type_hash_t *
livox_ros_driver2__msg__CustomPoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x32, 0x89, 0xac, 0x5e, 0x30, 0xc2, 0x15, 0xe0,
      0x55, 0xe9, 0xb4, 0x8f, 0xe0, 0xdf, 0x5c, 0x2f,
      0xb9, 0x16, 0xab, 0xbb, 0x0c, 0x3a, 0x75, 0x31,
      0x50, 0x71, 0xbb, 0x7d, 0x73, 0x50, 0x67, 0xf5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char livox_ros_driver2__msg__CustomPoint__TYPE_NAME[] = "livox_ros_driver2/msg/CustomPoint";

// Define type names, field names, and default values
static char livox_ros_driver2__msg__CustomPoint__FIELD_NAME__offset_time[] = "offset_time";
static char livox_ros_driver2__msg__CustomPoint__FIELD_NAME__x[] = "x";
static char livox_ros_driver2__msg__CustomPoint__FIELD_NAME__y[] = "y";
static char livox_ros_driver2__msg__CustomPoint__FIELD_NAME__z[] = "z";
static char livox_ros_driver2__msg__CustomPoint__FIELD_NAME__reflectivity[] = "reflectivity";
static char livox_ros_driver2__msg__CustomPoint__FIELD_NAME__tag[] = "tag";
static char livox_ros_driver2__msg__CustomPoint__FIELD_NAME__line[] = "line";

static rosidl_runtime_c__type_description__Field livox_ros_driver2__msg__CustomPoint__FIELDS[] = {
  {
    {livox_ros_driver2__msg__CustomPoint__FIELD_NAME__offset_time, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomPoint__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomPoint__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomPoint__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomPoint__FIELD_NAME__reflectivity, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomPoint__FIELD_NAME__tag, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomPoint__FIELD_NAME__line, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
livox_ros_driver2__msg__CustomPoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {livox_ros_driver2__msg__CustomPoint__TYPE_NAME, 33, 33},
      {livox_ros_driver2__msg__CustomPoint__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Livox costom pointcloud format.\n"
  "\n"
  "uint32 offset_time      # offset time relative to the base time\n"
  "float32 x               # X axis, unit:m\n"
  "float32 y               # Y axis, unit:m\n"
  "float32 z               # Z axis, unit:m\n"
  "uint8 reflectivity      # reflectivity, 0~255\n"
  "uint8 tag               # livox tag\n"
  "uint8 line              # laser number in lidar\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
livox_ros_driver2__msg__CustomPoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {livox_ros_driver2__msg__CustomPoint__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 353, 353},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
livox_ros_driver2__msg__CustomPoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *livox_ros_driver2__msg__CustomPoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
