# generated from rosidl_generator_py/resource/_idl.py.em
# with input from deltarobot_interfaces:msg/TrajectoryTask.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrajectoryTask(type):
    """Metaclass of message 'TrajectoryTask'."""

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
            module = import_type_support('deltarobot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'deltarobot_interfaces.msg.TrajectoryTask')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__trajectory_task
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__trajectory_task
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__trajectory_task
            cls._TYPE_SUPPORT = module.type_support_msg__msg__trajectory_task
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__trajectory_task

            from geometry_msgs.msg import Point32
            if Point32.__class__._TYPE_SUPPORT is None:
                Point32.__class__.__import_type_support__()

            from std_msgs.msg import String
            if String.__class__._TYPE_SUPPORT is None:
                String.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TrajectoryTask(metaclass=Metaclass_TrajectoryTask):
    """Message class 'TrajectoryTask'."""

    __slots__ = [
        '_pos_start',
        '_pos_end',
        '_task_time',
        '_task_type',
    ]

    _fields_and_field_types = {
        'pos_start': 'geometry_msgs/Point32',
        'pos_end': 'geometry_msgs/Point32',
        'task_time': 'float',
        'task_type': 'std_msgs/String',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'String'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Point32
        self.pos_start = kwargs.get('pos_start', Point32())
        from geometry_msgs.msg import Point32
        self.pos_end = kwargs.get('pos_end', Point32())
        self.task_time = kwargs.get('task_time', float())
        from std_msgs.msg import String
        self.task_type = kwargs.get('task_type', String())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.pos_start != other.pos_start:
            return False
        if self.pos_end != other.pos_end:
            return False
        if self.task_time != other.task_time:
            return False
        if self.task_type != other.task_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pos_start(self):
        """Message field 'pos_start'."""
        return self._pos_start

    @pos_start.setter
    def pos_start(self, value):
        if __debug__:
            from geometry_msgs.msg import Point32
            assert \
                isinstance(value, Point32), \
                "The 'pos_start' field must be a sub message of type 'Point32'"
        self._pos_start = value

    @builtins.property
    def pos_end(self):
        """Message field 'pos_end'."""
        return self._pos_end

    @pos_end.setter
    def pos_end(self, value):
        if __debug__:
            from geometry_msgs.msg import Point32
            assert \
                isinstance(value, Point32), \
                "The 'pos_end' field must be a sub message of type 'Point32'"
        self._pos_end = value

    @builtins.property
    def task_time(self):
        """Message field 'task_time'."""
        return self._task_time

    @task_time.setter
    def task_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'task_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'task_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._task_time = value

    @builtins.property
    def task_type(self):
        """Message field 'task_type'."""
        return self._task_type

    @task_type.setter
    def task_type(self, value):
        if __debug__:
            from std_msgs.msg import String
            assert \
                isinstance(value, String), \
                "The 'task_type' field must be a sub message of type 'String'"
        self._task_type = value
