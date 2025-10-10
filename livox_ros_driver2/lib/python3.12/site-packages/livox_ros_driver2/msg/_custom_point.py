# generated from rosidl_generator_py/resource/_idl.py.em
# with input from livox_ros_driver2:msg/CustomPoint.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CustomPoint(type):
    """Metaclass of message 'CustomPoint'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('livox_ros_driver2')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'livox_ros_driver2.msg.CustomPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__custom_point
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__custom_point
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__custom_point
            cls._TYPE_SUPPORT = module.type_support_msg__msg__custom_point
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__custom_point

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CustomPoint(metaclass=Metaclass_CustomPoint):
    """Message class 'CustomPoint'."""

    __slots__ = [
        '_offset_time',
        '_x',
        '_y',
        '_z',
        '_reflectivity',
        '_tag',
        '_line',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'offset_time': 'uint32',
        'x': 'float',
        'y': 'float',
        'z': 'float',
        'reflectivity': 'uint8',
        'tag': 'uint8',
        'line': 'uint8',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.offset_time = kwargs.get('offset_time', int())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.reflectivity = kwargs.get('reflectivity', int())
        self.tag = kwargs.get('tag', int())
        self.line = kwargs.get('line', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.offset_time != other.offset_time:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.reflectivity != other.reflectivity:
            return False
        if self.tag != other.tag:
            return False
        if self.line != other.line:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def offset_time(self):
        """Message field 'offset_time'."""
        return self._offset_time

    @offset_time.setter
    def offset_time(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'offset_time' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'offset_time' field must be an unsigned integer in [0, 4294967295]"
        self._offset_time = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z = value

    @builtins.property
    def reflectivity(self):
        """Message field 'reflectivity'."""
        return self._reflectivity

    @reflectivity.setter
    def reflectivity(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'reflectivity' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'reflectivity' field must be an unsigned integer in [0, 255]"
        self._reflectivity = value

    @builtins.property
    def tag(self):
        """Message field 'tag'."""
        return self._tag

    @tag.setter
    def tag(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'tag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'tag' field must be an unsigned integer in [0, 255]"
        self._tag = value

    @builtins.property
    def line(self):
        """Message field 'line'."""
        return self._line

    @line.setter
    def line(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'line' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'line' field must be an unsigned integer in [0, 255]"
        self._line = value
