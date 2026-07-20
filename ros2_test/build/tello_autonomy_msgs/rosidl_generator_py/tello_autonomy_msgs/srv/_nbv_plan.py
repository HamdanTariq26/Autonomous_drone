# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tello_autonomy_msgs:srv/NbvPlan.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NbvPlan_Request(type):
    """Metaclass of message 'NbvPlan_Request'."""

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
            module = import_type_support('tello_autonomy_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tello_autonomy_msgs.srv.NbvPlan_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__nbv_plan__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__nbv_plan__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__nbv_plan__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__nbv_plan__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__nbv_plan__request

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


class NbvPlan_Request(metaclass=Metaclass_NbvPlan_Request):
    """Message class 'NbvPlan_Request'."""

    __slots__ = [
        '_header',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())

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
        if self.header != other.header:
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
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NbvPlan_Response(type):
    """Metaclass of message 'NbvPlan_Response'."""

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
            module = import_type_support('tello_autonomy_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tello_autonomy_msgs.srv.NbvPlan_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__nbv_plan__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__nbv_plan__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__nbv_plan__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__nbv_plan__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__nbv_plan__response

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NbvPlan_Response(metaclass=Metaclass_NbvPlan_Response):
    """Message class 'NbvPlan_Response'."""

    __slots__ = [
        '_path',
    ]

    _fields_and_field_types = {
        'path': 'sequence<geometry_msgs/Pose>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.path = kwargs.get('path', [])

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
        if self.path != other.path:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def path(self):
        """Message field 'path'."""
        return self._path

    @path.setter
    def path(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
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
                 all(isinstance(v, Pose) for v in value) and
                 True), \
                "The 'path' field must be a set or sequence and each value of type 'Pose'"
        self._path = value


class Metaclass_NbvPlan(type):
    """Metaclass of service 'NbvPlan'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tello_autonomy_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tello_autonomy_msgs.srv.NbvPlan')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__nbv_plan

            from tello_autonomy_msgs.srv import _nbv_plan
            if _nbv_plan.Metaclass_NbvPlan_Request._TYPE_SUPPORT is None:
                _nbv_plan.Metaclass_NbvPlan_Request.__import_type_support__()
            if _nbv_plan.Metaclass_NbvPlan_Response._TYPE_SUPPORT is None:
                _nbv_plan.Metaclass_NbvPlan_Response.__import_type_support__()


class NbvPlan(metaclass=Metaclass_NbvPlan):
    from tello_autonomy_msgs.srv._nbv_plan import NbvPlan_Request as Request
    from tello_autonomy_msgs.srv._nbv_plan import NbvPlan_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
