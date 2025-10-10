// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from livox_ros_driver2:msg/CustomPoint.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "livox_ros_driver2/msg/detail/custom_point__struct.h"
#include "livox_ros_driver2/msg/detail/custom_point__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool livox_ros_driver2__msg__custom_point__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("livox_ros_driver2.msg._custom_point.CustomPoint", full_classname_dest, 47) == 0);
  }
  livox_ros_driver2__msg__CustomPoint * ros_message = _ros_message;
  {  // offset_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "offset_time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->offset_time = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y
    PyObject * field = PyObject_GetAttrString(_pymsg, "y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // reflectivity
    PyObject * field = PyObject_GetAttrString(_pymsg, "reflectivity");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->reflectivity = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // tag
    PyObject * field = PyObject_GetAttrString(_pymsg, "tag");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->tag = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // line
    PyObject * field = PyObject_GetAttrString(_pymsg, "line");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->line = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * livox_ros_driver2__msg__custom_point__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CustomPoint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("livox_ros_driver2.msg._custom_point");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CustomPoint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  livox_ros_driver2__msg__CustomPoint * ros_message = (livox_ros_driver2__msg__CustomPoint *)raw_ros_message;
  {  // offset_time
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->offset_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "offset_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reflectivity
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->reflectivity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reflectivity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tag
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->tag);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // line
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->line);
    {
      int rc = PyObject_SetAttrString(_pymessage, "line", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
