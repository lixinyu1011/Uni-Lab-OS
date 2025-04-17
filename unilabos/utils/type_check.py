import collections
import json
from typing import get_origin, get_args


def get_type_class(type_hint):
    origin = get_origin(type_hint)
    if origin is not None and issubclass(origin, collections.abc.Sequence):
        final_type = [get_args(type_hint)[0]]  # 默认sequence中类型都一样
    else:
        final_type = type_hint
    return final_type


class TypeEncoder(json.JSONEncoder):
    """自定义JSON编码器处理特殊类型"""

    def default(self, obj):
        # 优先处理类型对象
        if isinstance(obj, type):
            return str(obj)[8:-2]
        return super().default(obj)

