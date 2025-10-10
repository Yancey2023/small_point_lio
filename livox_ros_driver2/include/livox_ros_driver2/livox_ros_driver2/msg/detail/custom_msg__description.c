// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from livox_ros_driver2:msg/CustomMsg.idl
// generated code does not contain a copyright notice

#include "livox_ros_driver2/msg/detail/custom_msg__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_livox_ros_driver2
const rosidl_type_hash_t *
livox_ros_driver2__msg__CustomMsg__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2e, 0xb1, 0x7e, 0x5e, 0x35, 0xbf, 0x5a, 0x2d,
      0x6c, 0xd0, 0x79, 0xf3, 0xc5, 0x7f, 0x33, 0xab,
      0xf9, 0x5e, 0x5b, 0xf7, 0xd7, 0x1b, 0x42, 0xcd,
      0xdc, 0x05, 0xda, 0xac, 0xe7, 0x3d, 0x81, 0x5a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "livox_ros_driver2/msg/detail/custom_point__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t livox_ros_driver2__msg__CustomPoint__EXPECTED_HASH = {1, {
    0x32, 0x89, 0xac, 0x5e, 0x30, 0xc2, 0x15, 0xe0,
    0x55, 0xe9, 0xb4, 0x8f, 0xe0, 0xdf, 0x5c, 0x2f,
    0xb9, 0x16, 0xab, 0xbb, 0x0c, 0x3a, 0x75, 0x31,
    0x50, 0x71, 0xbb, 0x7d, 0x73, 0x50, 0x67, 0xf5,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char livox_ros_driver2__msg__CustomMsg__TYPE_NAME[] = "livox_ros_driver2/msg/CustomMsg";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char livox_ros_driver2__msg__CustomPoint__TYPE_NAME[] = "livox_ros_driver2/msg/CustomPoint";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char livox_ros_driver2__msg__CustomMsg__FIELD_NAME__header[] = "header";
static char livox_ros_driver2__msg__CustomMsg__FIELD_NAME__timebase[] = "timebase";
static char livox_ros_driver2__msg__CustomMsg__FIELD_NAME__point_num[] = "point_num";
static char livox_ros_driver2__msg__CustomMsg__FIELD_NAME__lidar_id[] = "lidar_id";
static char livox_ros_driver2__msg__CustomMsg__FIELD_NAME__rsvd[] = "rsvd";
static char livox_ros_driver2__msg__CustomMsg__FIELD_NAME__points[] = "points";

static rosidl_runtime_c__type_description__Field livox_ros_driver2__msg__CustomMsg__FIELDS[] = {
  {
    {livox_ros_driver2__msg__CustomMsg__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomMsg__FIELD_NAME__timebase, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomMsg__FIELD_NAME__point_num, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomMsg__FIELD_NAME__lidar_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomMsg__FIELD_NAME__rsvd, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomMsg__FIELD_NAME__points, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {livox_ros_driver2__msg__CustomPoint__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription livox_ros_driver2__msg__CustomMsg__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {livox_ros_driver2__msg__CustomPoint__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
livox_ros_driver2__msg__CustomMsg__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {livox_ros_driver2__msg__CustomMsg__TYPE_NAME, 31, 31},
      {livox_ros_driver2__msg__CustomMsg__FIELDS, 6, 6},
    },
    {livox_ros_driver2__msg__CustomMsg__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&livox_ros_driver2__msg__CustomPoint__EXPECTED_HASH, livox_ros_driver2__msg__CustomPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = livox_ros_driver2__msg__CustomPoint__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Livox publish pointcloud msg format.\n"
  "\n"
  "std_msgs/Header header    # ROS standard message header\n"
  "uint64 timebase           # The time of first point\n"
  "uint32 point_num          # Total number of pointclouds\n"
  "uint8  lidar_id           # Lidar device id number\n"
  "uint8[3]  rsvd            # Reserved use\n"
  "CustomPoint[] points      # Pointcloud data\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
livox_ros_driver2__msg__CustomMsg__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {livox_ros_driver2__msg__CustomMsg__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 341, 341},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
livox_ros_driver2__msg__CustomMsg__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *livox_ros_driver2__msg__CustomMsg__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *livox_ros_driver2__msg__CustomPoint__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
