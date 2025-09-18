#!/usr/bin/env python3
"""
将 plr_resources_export_new.json 转换为 Bioyond JSON（data 列表结构）。
输出文件名：plr_to_bioyond_result_new.json
- 尺寸与坐标均取自输入 PLR 项的 size_* 与 location.*
- Well 子项（如配液瓶(小)）转为父容器 detail 中的条目
- 透传 location_info（若存在）
"""
import json
import os
from typing import Any, Dict, List

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PLR_INPUT = os.path.join(BASE_DIR, 'plr_resources_export_new.json')
BIOYOND_OUTPUT = os.path.join(BASE_DIR, 'plr_to_bioyond_result_new.json')


def as_int(v: Any) -> int:
    try:
        return int(v)
    except Exception:
        try:
            return int(float(v))
        except Exception:
            return 0


def make_location_entry(loc: Dict[str, Any]) -> Dict[str, Any]:
    return {
        'id': None,
        'whid': None,
        'whName': None,
        'code': None,
        'x': as_int((loc or {}).get('x', 0)),
        'y': as_int((loc or {}).get('y', 0)),
        'z': as_int((loc or {}).get('z', 0)),
        'quantity': 0,
    }


def extract_detail_from_child(child: Dict[str, Any]) -> Dict[str, Any]:
    loc = child.get('location') or {}
    return {
        'id': child.get('name'),
        'detailMaterialId': None,
        'code': None,
        'name': child.get('name'),
        'quantity': '1',
        'unit': '个',
        'x': as_int(loc.get('x', 0)),
        'y': as_int(loc.get('y', 0)),
        'z': as_int(loc.get('z', 0)),
        'associateId': None,
    }


def plr_to_bioyond(plr_list: List[Dict[str, Any]]) -> Dict[str, Any]:
    bioyond_items: List[Dict[str, Any]] = []

    # 建立 name → plr item，便于引用父子
    name_to_plr = {it.get('name'): it for it in plr_list}

    for item in plr_list:
        name = item.get('name')
        loc = item.get('location') or {}
        cfg_locinfo = item.get('location_info')  # 可能存在，直接透传
        type_name = None

        # 基于 PLR 类型推断 Bioyond 的 typeName（可按需扩展映射）
        plr_type = item.get('type')
        if plr_type in ('TipBox64', 'TipRack', 'Container') and name and ('枪头' in name or plr_type == 'TipBox64'):
            type_name = '枪头盒'
        elif plr_type in ('BottleRack',):
            type_name = '配液瓶(小)板'
        elif plr_type in ('Block',):
            type_name = '适配器块'
        elif plr_type in ('Bottle', 'Well'):
            # 此类作为子项写入 detail，不在顶层生成条目
            continue
        else:
            # 兜底
            type_name = '容器'

        bioyond_obj: Dict[str, Any] = {
            'id': name,  # 如需可改为 UUID
            'typeName': type_name,
            'code': None,
            'barCode': None,
            'name': name,
            'quantity': 1,
            'lockQuantity': 0,
            'unit': '个',
            'status': 1,
            'isUse': False,
            'locations': [make_location_entry(loc)],
            'detail': [],
        }

        # 透传位置信息（若存在）
        if isinstance(cfg_locinfo, dict):
            # Bioyond 顶层无该字段，按现有示例只在 locations 中呈现
            # 如需保留，可添加到扩展字段，这里选择忽略，避免 schema 偏差
            pass

        # 处理子资源作为 detail（比如 Bottle / Well）
        children = item.get('children') or []
        for ch in children:
            # 输入 children 可能是内嵌对象或名称字符串
            ch_obj = ch if isinstance(ch, dict) else name_to_plr.get(ch)
            if not isinstance(ch_obj, dict):
                continue
            if ch_obj.get('type') in ('Bottle', 'Well'):
                bioyond_obj['detail'].append(extract_detail_from_child(ch_obj))

        bioyond_items.append(bioyond_obj)

    return {
        'data': bioyond_items,
        'code': 1,
        'message': '',
        'timestamp': 0,
    }


def main():
    if not os.path.exists(PLR_INPUT):
        print(f'❌ 未找到输入文件: {PLR_INPUT}')
        return
    with open(PLR_INPUT, 'r', encoding='utf-8') as f:
        plr_list = json.load(f)

    bioyond_json = plr_to_bioyond(plr_list)

    with open(BIOYOND_OUTPUT, 'w', encoding='utf-8') as f:
        json.dump(bioyond_json, f, ensure_ascii=False, indent=2)
    print(f'✅ 已生成: {BIOYOND_OUTPUT}')


if __name__ == '__main__':
    main()
