// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from deadwheel_msgs:msg/DeadwheelTicks.idl
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
#include "deadwheel_msgs/msg/detail/deadwheel_ticks__struct.h"
#include "deadwheel_msgs/msg/detail/deadwheel_ticks__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool deadwheel_msgs__msg__deadwheel_ticks__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("deadwheel_msgs.msg._deadwheel_ticks.DeadwheelTicks", full_classname_dest, 50) == 0);
  }
  deadwheel_msgs__msg__DeadwheelTicks * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // t0
    PyObject * field = PyObject_GetAttrString(_pymsg, "t0");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->t0 = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // t1
    PyObject * field = PyObject_GetAttrString(_pymsg, "t1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->t1 = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // t2
    PyObject * field = PyObject_GetAttrString(_pymsg, "t2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->t2 = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * deadwheel_msgs__msg__deadwheel_ticks__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DeadwheelTicks */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("deadwheel_msgs.msg._deadwheel_ticks");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DeadwheelTicks");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  deadwheel_msgs__msg__DeadwheelTicks * ros_message = (deadwheel_msgs__msg__DeadwheelTicks *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t0
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->t0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t1
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->t1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t2
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->t2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
