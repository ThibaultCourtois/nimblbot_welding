# generated from rosidl_generator_py/resource/_idl.py.em
# with input from telesoud_msgs:msg/TelesoudInstruction.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'pose1'
# Member 'speed_vector'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TelesoudInstruction(type):
    """Metaclass of message 'TelesoudInstruction'."""

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
            module = import_type_support('telesoud_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'telesoud_msgs.msg.TelesoudInstruction')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__telesoud_instruction
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__telesoud_instruction
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__telesoud_instruction
            cls._TYPE_SUPPORT = module.type_support_msg__msg__telesoud_instruction
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__telesoud_instruction

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TelesoudInstruction(metaclass=Metaclass_TelesoudInstruction):
    """Message class 'TelesoudInstruction'."""

    __slots__ = [
        '_instruction_code',
        '_pose1',
        '_speed_vector',
        '_speed',
        '_free_drive_btn_status',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'instruction_code': 'uint16',
        'pose1': 'double[6]',
        'speed_vector': 'double[6]',
        'speed': 'double',
        'free_drive_btn_status': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 6),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.instruction_code = kwargs.get('instruction_code', int())
        if 'pose1' not in kwargs:
            self.pose1 = numpy.zeros(6, dtype=numpy.float64)
        else:
            self.pose1 = numpy.array(kwargs.get('pose1'), dtype=numpy.float64)
            assert self.pose1.shape == (6, )
        if 'speed_vector' not in kwargs:
            self.speed_vector = numpy.zeros(6, dtype=numpy.float64)
        else:
            self.speed_vector = numpy.array(kwargs.get('speed_vector'), dtype=numpy.float64)
            assert self.speed_vector.shape == (6, )
        self.speed = kwargs.get('speed', float())
        self.free_drive_btn_status = kwargs.get('free_drive_btn_status', bool())

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
        if self.instruction_code != other.instruction_code:
            return False
        if all(self.pose1 != other.pose1):
            return False
        if all(self.speed_vector != other.speed_vector):
            return False
        if self.speed != other.speed:
            return False
        if self.free_drive_btn_status != other.free_drive_btn_status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def instruction_code(self):
        """Message field 'instruction_code'."""
        return self._instruction_code

    @instruction_code.setter
    def instruction_code(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'instruction_code' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'instruction_code' field must be an unsigned integer in [0, 65535]"
        self._instruction_code = value

    @builtins.property
    def pose1(self):
        """Message field 'pose1'."""
        return self._pose1

    @pose1.setter
    def pose1(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'pose1' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 6, \
                    "The 'pose1' numpy.ndarray() must have a size of 6"
                self._pose1 = value
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'pose1' field must be a set or sequence with length 6 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._pose1 = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def speed_vector(self):
        """Message field 'speed_vector'."""
        return self._speed_vector

    @speed_vector.setter
    def speed_vector(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'speed_vector' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 6, \
                    "The 'speed_vector' numpy.ndarray() must have a size of 6"
                self._speed_vector = value
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'speed_vector' field must be a set or sequence with length 6 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._speed_vector = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._speed = value

    @builtins.property
    def free_drive_btn_status(self):
        """Message field 'free_drive_btn_status'."""
        return self._free_drive_btn_status

    @free_drive_btn_status.setter
    def free_drive_btn_status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'free_drive_btn_status' field must be of type 'bool'"
        self._free_drive_btn_status = value
