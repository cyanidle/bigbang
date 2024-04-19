from copy import copy, deepcopy
from dataclasses import dataclass
import json
from typing import Any, Deque, Dict, Generic, Iterable, Iterator, List, MutableMapping, Optional, Sequence, Tuple, Type, TypeVar, Union, cast
from typing_extensions import Self
from collections import deque
from pydantic import Json

__all__ = ("FlatDict", "merge_flat_dicts", "JsonDict", "JsonDictIterator", "JsonKey", "JsonDictIteratorSimple")

JsonKey = Union[Sequence[str], str]
JsonItem = Union[str, None, int, float, list, dict, Any]
FlatDict = Dict[str, JsonItem]
_T = TypeVar("_T")

def merge_flat_dicts(first: FlatDict, second: FlatDict) -> FlatDict:
    return {**first, **second}

class _JsonDictKeyPart:
    __slots__ = ["key", "as_index"]
    def __init__(self, raw: str) -> None:
        self.key = raw
        self.as_index: int = cast(int, None)
        if len(raw) >= 3:
            is_braced = raw[0] == "[" and raw[-1] == "]"
            if is_braced:
                try:
                    self.as_index = int(raw[1:-1])
                except:
                    pass
    @property
    def wanted_container(self) -> type:
        return list if self.is_index else dict
    def is_in(self, target: Union[dict, list]):
        if isinstance(target, dict):
            if self.is_index:
                raise KeyError("Attempt to get by index in dict!")
            return self.key in target
        elif isinstance(target, list):
            if not self.is_index:
                raise KeyError("Attempt to get by key in list!")
            return self.as_index < len(target)
        else:
            return TypeError("Attempt to shadow already existing value")
    def set_in(self, target: Union[dict, list], value: Any):
        if isinstance(target, dict):
            if self.is_index:
                raise KeyError("Attempt to set by index in dict!")
            target[self.key] = value
        elif isinstance(target, list):
            if not self.is_index:
                raise KeyError("Attempt to set by key in list!")
            while len(target) <= self.as_index:
                target.append(None)
            target[self.as_index] = value
        else:
            raise TypeError
    def get_from(self, target: Union[dict, list]):
        if isinstance(target, dict) and not self.is_index:
            return target[self.key]
        elif isinstance(target, list) and self.is_index:
            return target[self.as_index]
        else:
            raise KeyError(f"Incompatible key {self.key} with {target.__class__.__name__}:{target}")
    @property
    def is_index(self):
        return self.as_index is not None

class _JsonDictKey:
    __slots__ = ["subkeys"]
    def __init__(self, keys: Union[Sequence[str], str]) -> None:
        if isinstance(keys, str):
            keys = keys.split(":")
        self.subkeys = tuple((_JsonDictKeyPart(subkey) for subkey in keys))
    def __iter__(self) -> Iterator[_JsonDictKeyPart]:
        return self.subkeys.__iter__()

class JsonDict(MutableMapping[JsonKey, JsonItem]):
    __slots__ = ["_dict"]
    def __init__(self, src_dict: Union[FlatDict, dict, Any] = None, **kwargs) -> None:
        if src_dict is None: src_dict = kwargs
        if not isinstance(src_dict, dict):
            raise TypeError("Can only construct JsonDict from dict!")
        self._dict = src_dict
        if self._dict:
            self.nest()
    @property
    def top(self):
        return self._dict
    def nest(self, sep: str = ":"):
        copy = JsonDict()
        for k, v in self._dict.items():
            if isinstance(v, list):
                for item in v:
                    if isinstance(item, dict):
                        item = JsonDict(item)
            copy[tuple(k.split(sep))] = v
        self._dict = copy._dict
    def flattened(self, sep:str = ":") -> FlatDict:
        result = {}
        for iter in self.full_iter():
            result[sep.join(iter.key())] = iter.value()
        return result
    def as_bytes(self, encoding:str = "utf-8"):
        return json.dumps(self._dict, separators=(',', ':')).encode(encoding=encoding)
    @classmethod
    def from_bytes(cls, raw: bytes, encoding:str = "utf-8"):
        return cls(json.loads(raw.decode(encoding=encoding)))
    def __eq__(self, other: Self) -> bool:
        if len(self._dict) != len(other._dict):
            return False
        for iter in self.full_iter():
            try:
                if iter.value() != other[iter.key()]: return False
            except: return False
        for iter in other.full_iter():
            try:
                if iter.value() != self[iter.key()]: return False
            except: return False
        return True
    def __contains__(self, key: Union[JsonKey, 'JsonDictIterator']) -> bool:
        try:
            if isinstance(key, JsonDictIterator):
                return self.__contains__(key.key())
            _ = self[key]
            return True
        except:
            return False
    def __delitem__(self, key: Union[JsonKey, 'JsonDictIterator']) -> None:
        if isinstance(key, JsonDictIterator):
            self.__delitem__(key.key())
            return
        keys = _JsonDictKey(key)
        current: Union[List, Dict] = self._dict
        for subkey in keys.subkeys[:-1]:
            current = subkey.get_from(current)
        last = keys.subkeys[-1]
        if last.is_index:
            del cast(list, current)[last.as_index]
        else:
            del cast(dict, current)[last.key]
    def get(self, key: Union[JsonKey, 'JsonDictIterator'], default:_T = None) -> Union[JsonItem, _T]:
        try:
            if isinstance(key, JsonDictIterator):
                return self.get(key.key())
            return self.__getitem__(key)
        except:
            return default
    def __getitem__(self, key: Union[JsonKey, 'JsonDictIterator']) -> JsonItem:
        if isinstance(key, JsonDictIterator):
            return self.__getitem__(key.key())
        keys = _JsonDictKey(key)
        current = self._dict
        for subkey in keys:
            current = subkey.get_from(current)
        return current
    def __setitem__(self, key: Union[JsonKey, 'JsonDictIterator'], value: Any) -> None:
        if isinstance(key, JsonDictIterator):
            self.__setitem__(key.key(), value)
            return
        if isinstance(value, JsonDict):
            value = value._dict
        keys = _JsonDictKey(key)
        current: Union[list, dict] = self._dict
        for num, subkey in enumerate(keys.subkeys[:-1]):
            if not subkey.is_in(current) or not isinstance(subkey.get_from(current), (dict, list)):
                subkey.set_in(current, keys.subkeys[num + 1].wanted_container())
            current = subkey.get_from(current)
        last = keys.subkeys[-1]
        if last.is_index:
            while len(current) <= last.as_index:
                cast(list, current).append(None)
            cast(list, current)[last.as_index] = value
        else:
            cast(dict, current)[last.key] = value
    def __len__(self) -> int:
        num = 0
        for _ in self:
            num += 1
        return num
    def full_iter(self) -> 'JsonDictIterator':
        return JsonDictIterator(source=self)
    def __iter__(self) -> 'JsonDictIteratorSimple':
        return JsonDictIteratorSimple(source = self)
    def __str__(self) -> str:
        return str(self._dict)
    def __repr__(self) -> str:
        return repr(self._dict)

@dataclass
class _TraverseState:
    iter: Union[Iterator[str], Iterator[JsonItem]]
    key: Optional[str]
    container: Union[dict, list]
    count: int
    value: JsonItem

class JsonDictIterator(Iterator[JsonItem]):
    __slots__ = ["_history", "_state"]
    def __init__(self, *, source: JsonDict) -> None:
        self._state = _TraverseState(
            source._dict.__iter__(),
            None,
            source._dict,
            -1,
            None
        )
        self._history: Deque[_TraverseState] = deque()
    def value(self) -> JsonItem:
        return self._state.value
    def field(self) -> str:
        return cast(str, self.__key())
    def domain(self) -> JsonKey:
        return cast(JsonKey, tuple(map(lambda state: state.key, self._history)))
    def key(self) -> JsonKey:
        return cast(JsonKey, tuple(map(lambda state: state.key, self._history)) + (self.__key(),))
    def depth(self) -> int:
        return len(self._history) + 1
    def __str__(self) -> str:
        return ":".join(self.key())
    def __repr__(self) -> str:
        return f"JsonDictIterator ({':'.join(self.key())})"
    def __key(self):
        if isinstance(self._state.container, dict):
            return self._state.key
        else:
            return f"[{self._state.count}]"
    def __advance(self):
        if isinstance(self._state.container, dict):
            self._state.key = cast(str, self._state.iter.__next__())
            self._state.value = self._state.container[self._state.key]
        else:
            self._state.count += 1
            self._state.key = f"[{self._state.count}]"
            self._state.value = self._state.iter.__next__() 
    def __pop_state(self):
        self._state = self._history.pop()
    def __push_state(self):
        self._history.append(copy(self._state))
        self._state.container = cast(Union[dict, list], self.value())
        self._state.iter = self._state.container.__iter__()
        self._state.count = -1
    def __iter__(self):
        return self
    def __next__(self) -> Self:
        try:
            self.__advance()
            if isinstance(self._state.value, (dict, list)):
                self.__push_state()
                if not self._state.container and self._history:
                    self.__pop_state()
                self.__next__()
        except StopIteration:
            if not self._history:
                raise StopIteration
            else:
                self.__pop_state()
                self.__next__()
        return self
    
class JsonDictIteratorSimple(JsonDictIterator):
    def __next__(self) -> str:
        super().__next__()
        return str(self)

def test():
    a = JsonDict()
    a["1"] = 1
    a[("2", "3")] = 2
    a[("2", "4", "[0]")] = 3
    a[("2", "4", "[2]")] = 4
    a[("2", "4", "[1]", "3", "[10]", "2")] = 5
    assert a["1"] == 1
    assert a[("2", "3")] == 2
    assert a[("2", "4", "[0]")] == 3
    assert a[("2", "4", "[2]")] == 4
    assert a[("2", "4", "[1]", "3", "[10]", "2")] == 5
    flat = a.flattened()
    nested = JsonDict(flat)
    assert nested == a
    b = JsonDict({"a": 1, "b:c:d": 3})
    print({**b})
    print("Test Passed!")

if __name__ == "__main__":
    test()