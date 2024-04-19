from copy import copy, deepcopy
from dataclasses import MISSING, Field, dataclass, fields, is_dataclass
from typing import Any, Dict, Union, get_args, get_origin

import debugpy
import rospy

# use field(default_factory=() for nested fields)
@dataclass
class RosparamsDataclass:

    def __post_init__(self):
        if not is_dataclass(self):
            raise RuntimeError(f"'{self.__class__.__name__}' must be a dataclass!")
        for field in fields(self):
            if field.default is MISSING and field.default_factory is MISSING and field.init:
                raise RuntimeError(f"'{self.__class__.__name__} ({field.name})': Must provide defaults for every field of RosparamsDataclass subclass!")

    def update(self, name: Union[str, Field, None], *, prefix = ""):
        def find_field(name: str) -> Field:
            for subfield in fields(self):
                if subfield.name == name:
                    return subfield
            rospy.logerr(f"Field {name} not found!")
            return None
        if name is None or (isinstance(name, str) and not name):
            self.update_all()
            return
        elif isinstance(name, Field):
            field = name
        elif isinstance(name, str):
            rospy.loginfo(f"Updating {name}...")
            field = find_field(name)
            if field is None: return
        else:
            raise TypeError(f"Typeof param: {name.__class__.__name__}")
        if not field.init:
            return
        try:
            try:
                if field.default is not MISSING:
                    default = field.default 
                elif field.default_factory is not MISSING:
                    default = field.default_factory()
                else:
                    raise ValueError
            except:
                default = None
            field_type_stripped = get_origin(field.type) or field.type
            is_nested = issubclass(field_type_stripped, RosparamsDataclass)
            if is_nested:
                value: RosparamsDataclass = getattr(self, field.name) or field_type_stripped()
                if value is None:
                    raise RuntimeError(f"Nested Rosparams class is missing or invalid type!")
                if prefix:
                    actual_prefix = f"{prefix}{field.name}/" 
                else:
                    actual_prefix = f"{field.name}/"
                rospy.loginfo(f"Updating {field.name} ({value.__class__.__name__})")
                value.update_all(prefix=actual_prefix)
                return
            else:
                full_path = f"~{prefix}{field.name}"
                try:
                    value = rospy.get_param(full_path)
                except:
                    rospy.logwarn(f"Node: ({rospy.get_name()}): Rosparam missing: ~{prefix}{field.name} (type: {field_type_stripped.__name__}). Setting default: {default}")
                    if default is not None: 
                        rospy.set_param(full_path, default)
                    value = default
                if not isinstance(value, field_type_stripped):
                    try:
                        if field_type_stripped in (list, dict):
                            raise TypeError("Type is too complex")
                        value = field_type_stripped(value)
                    except TypeError:
                        debugpy.breakpoint()
                        rospy.logwarn(f"Incorrect type recieved for rosparam with name: '{field.name}'")
                        rospy.logwarn(f"Received: '{type(value)}'| Wanted: '{field.type}'")
                        rospy.logwarn(f"Path to Param: '{full_path}'")
                        raise
                setattr(self, field.name, copy(value))
        except:
            rospy.logwarn(f"While parsing field '{field.name}'({field.type.__name__})")
            raise

    def update_all(self, *, prefix = ""):
        if not is_dataclass(self):
            raise RuntimeError(f"'{self.__class__.__name__}' must be a dataclass!")
        for field in fields(self):
            self.update(field, prefix=prefix)
        self.__post_init__()