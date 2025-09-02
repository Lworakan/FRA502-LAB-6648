# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller_interfaces:srv/SetMaxPizza.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetMaxPizza_Request(type):
    """Metaclass of message 'SetMaxPizza_Request'."""

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
            module = import_type_support('controller_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller_interfaces.srv.SetMaxPizza_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_max_pizza__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_max_pizza__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_max_pizza__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_max_pizza__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_max_pizza__request

            from std_msgs.msg import Int64
            if Int64.__class__._TYPE_SUPPORT is None:
                Int64.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetMaxPizza_Request(metaclass=Metaclass_SetMaxPizza_Request):
    """Message class 'SetMaxPizza_Request'."""

    __slots__ = [
        '_max_pizza',
    ]

    _fields_and_field_types = {
        'max_pizza': 'std_msgs/Int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Int64
        self.max_pizza = kwargs.get('max_pizza', Int64())

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
        if self.max_pizza != other.max_pizza:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def max_pizza(self):
        """Message field 'max_pizza'."""
        return self._max_pizza

    @max_pizza.setter
    def max_pizza(self, value):
        if __debug__:
            from std_msgs.msg import Int64
            assert \
                isinstance(value, Int64), \
                "The 'max_pizza' field must be a sub message of type 'Int64'"
        self._max_pizza = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetMaxPizza_Response(type):
    """Metaclass of message 'SetMaxPizza_Response'."""

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
            module = import_type_support('controller_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller_interfaces.srv.SetMaxPizza_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_max_pizza__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_max_pizza__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_max_pizza__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_max_pizza__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_max_pizza__response

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


class SetMaxPizza_Response(metaclass=Metaclass_SetMaxPizza_Response):
    """Message class 'SetMaxPizza_Response'."""

    __slots__ = [
        '_log',
    ]

    _fields_and_field_types = {
        'log': 'std_msgs/String',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'String'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import String
        self.log = kwargs.get('log', String())

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
        if self.log != other.log:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def log(self):
        """Message field 'log'."""
        return self._log

    @log.setter
    def log(self, value):
        if __debug__:
            from std_msgs.msg import String
            assert \
                isinstance(value, String), \
                "The 'log' field must be a sub message of type 'String'"
        self._log = value


class Metaclass_SetMaxPizza(type):
    """Metaclass of service 'SetMaxPizza'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('controller_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller_interfaces.srv.SetMaxPizza')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_max_pizza

            from controller_interfaces.srv import _set_max_pizza
            if _set_max_pizza.Metaclass_SetMaxPizza_Request._TYPE_SUPPORT is None:
                _set_max_pizza.Metaclass_SetMaxPizza_Request.__import_type_support__()
            if _set_max_pizza.Metaclass_SetMaxPizza_Response._TYPE_SUPPORT is None:
                _set_max_pizza.Metaclass_SetMaxPizza_Response.__import_type_support__()


class SetMaxPizza(metaclass=Metaclass_SetMaxPizza):
    from controller_interfaces.srv._set_max_pizza import SetMaxPizza_Request as Request
    from controller_interfaces.srv._set_max_pizza import SetMaxPizza_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
