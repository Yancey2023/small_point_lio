# generated from rosidl_generator_py/resource/_idl.py.em
# with input from livox_ros_driver2:msg/CustomMsg.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'rsvd'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CustomMsg(type):
    """Metaclass of message 'CustomMsg'."""

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
                'livox_ros_driver2.msg.CustomMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__custom_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__custom_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__custom_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__custom_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__custom_msg

            from livox_ros_driver2.msg import CustomPoint
            if CustomPoint.__class__._TYPE_SUPPORT is None:
                CustomPoint.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CustomMsg(metaclass=Metaclass_CustomMsg):
    """Message class 'CustomMsg'."""

    __slots__ = [
        '_header',
        '_timebase',
        '_point_num',
        '_lidar_id',
        '_rsvd',
        '_points',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'timebase': 'uint64',
        'point_num': 'uint32',
        'lidar_id': 'uint8',
        'rsvd': 'uint8[3]',
        'points': 'sequence<livox_ros_driver2/CustomPoint>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 3),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['livox_ros_driver2', 'msg'], 'CustomPoint')),  # noqa: E501
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
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.timebase = kwargs.get('timebase', int())
        self.point_num = kwargs.get('point_num', int())
        self.lidar_id = kwargs.get('lidar_id', int())
        if 'rsvd' not in kwargs:
            self.rsvd = numpy.zeros(3, dtype=numpy.uint8)
        else:
            self.rsvd = numpy.array(kwargs.get('rsvd'), dtype=numpy.uint8)
            assert self.rsvd.shape == (3, )
        self.points = kwargs.get('points', [])

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
        if self.header != other.header:
            return False
        if self.timebase != other.timebase:
            return False
        if self.point_num != other.point_num:
            return False
        if self.lidar_id != other.lidar_id:
            return False
        if any(self.rsvd != other.rsvd):
            return False
        if self.points != other.points:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def timebase(self):
        """Message field 'timebase'."""
        return self._timebase

    @timebase.setter
    def timebase(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'timebase' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timebase' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timebase = value

    @builtins.property
    def point_num(self):
        """Message field 'point_num'."""
        return self._point_num

    @point_num.setter
    def point_num(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'point_num' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'point_num' field must be an unsigned integer in [0, 4294967295]"
        self._point_num = value

    @builtins.property
    def lidar_id(self):
        """Message field 'lidar_id'."""
        return self._lidar_id

    @lidar_id.setter
    def lidar_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'lidar_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'lidar_id' field must be an unsigned integer in [0, 255]"
        self._lidar_id = value

    @builtins.property
    def rsvd(self):
        """Message field 'rsvd'."""
        return self._rsvd

    @rsvd.setter
    def rsvd(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.uint8, \
                    "The 'rsvd' numpy.ndarray() must have the dtype of 'numpy.uint8'"
                assert value.size == 3, \
                    "The 'rsvd' numpy.ndarray() must have a size of 3"
                self._rsvd = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'rsvd' field must be a set or sequence with length 3 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._rsvd = numpy.array(value, dtype=numpy.uint8)

    @builtins.property
    def points(self):
        """Message field 'points'."""
        return self._points

    @points.setter
    def points(self, value):
        if self._check_fields:
            from livox_ros_driver2.msg import CustomPoint
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, CustomPoint) for v in value) and
                 True), \
                "The 'points' field must be a set or sequence and each value of type 'CustomPoint'"
        self._points = value
