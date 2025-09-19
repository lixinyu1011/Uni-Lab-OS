#!/usr/bin/env python3
"""
将 bioyond_test_yibin3_unilab_result_new.json 转为与 plr_resources_export_new.json 对齐的 PLR 结构。
输出：bioyond_to_plr_result_new.json
"""
import json
import os
from typing import Any, Dict, List

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BIOYOND_INPUT = os.path.join(BASE_DIR, 'bioyond_test_yibin3_unilab_result_new.json')
PLR_OUTPUT = os.path.join(BASE_DIR, 'bioyond_to_plr_result_new.json')

# 坐标不再写死：一律从输入 position 读取；若缺失则保持为 0


def extract_location_info(cfg: Dict[str, Any]) -> Dict[str, Any] | None:
    li = cfg.get('location_info') if isinstance(cfg, dict) else None
    if not isinstance(li, dict):
        return None
    return {
        'location_id': li.get('location_id'),
        'warehouse_id': li.get('warehouse_id'),
        'warehouse_name': li.get('warehouse_name'),
        'code': li.get('code'),
        'quantity_at_location': li.get('quantity_at_location'),
    }


def get_size_from_bioyond_wuliao(resource_name: str, resource_class: str) -> Dict[str, float] | None:
    """
    根据 resource_class 从 Bioyond_wuliao.py 中动态获取尺寸信息
    使用统一的配置字典，无需硬编码类名
    """
    try:
        # 动态导入 Bioyond_wuliao 模块
        import sys
        import os
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        
        from Bioyond_wuliao import get_resource_size
        
        # 直接使用配置函数获取尺寸
        return get_resource_size(resource_class)
            
    except ImportError as e:
        print(f"⚠️ 无法导入 Bioyond_wuliao 模块: {e}")
        return None
    except Exception as e:
        print(f"⚠️ 获取尺寸时出错: {e}")
        return None


def convert_item(item: Dict[str, Any]) -> Dict[str, Any]:
    name = item.get('name')
    cls = item.get('class', '') or ''
    pos = item.get('position') or {}
    cfg = item.get('config') or {}

    base = {
        'name': name,
        # 尺寸后续按输入读取并补充
        'size_x': None,
        'size_y': None,
        'size_z': None,
        'location': {
            'x': int(pos.get('x', 0) or 0),
            'y': int(pos.get('y', 0) or 0),
            'z': int(pos.get('z', 0) or 0),
            'type': 'Coordinate',
        },
        'rotation': {'x': 0, 'y': 0, 'z': 0, 'type': 'Rotation'},
        'category': 'Container',
        'model': None,
        'barcode': None,
        'children': [],
        'parent_name': None,
        'max_volume': None,
        'material_z_thickness': None,
        'compute_volume_from_height': None,
        'compute_height_from_volume': None,
    }

    # 附加 location_info
    li = extract_location_info(cfg)
    if li is not None:
        base['location_info'] = li

    # 根据 class 从 Bioyond_wuliao.py 获取尺寸
    size = get_size_from_bioyond_wuliao(name, cls)
    if size is not None:
        base.update(size)

    # 分类映射，仅决定 type 与 category，不再覆盖坐标
    if cls == '枪头盒':
        base['type'] = 'TipBox64'
        return base

    if cls in ('配液瓶小板', '配液瓶(小)板'):
        base['type'] = 'BottleRack'
        return base

    if cls in ('配液瓶小', '配液瓶(小)'):
        base['type'] = 'Bottle'
        base['category'] = 'Well'
        # 父关系从输入的 parent 字段获取（汇总阶段处理）
        parent = item.get('parent')
        if parent:
            base['parent_name'] = None
        return base

    if cls == '适配器块':
        base['type'] = 'Block'
        return base

    base['type'] = 'Container'
    return base


def bioyond_to_plr(bioyond_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    prelim = [convert_item(it) for it in bioyond_list]

    # 建立父映射
    name_to_prelim = {it['name']: it for it in prelim}

    result: List[Dict[str, Any]] = []
    for it in prelim:
        if it['type'] == 'Bottle':
            # 优先根据输入 parent 名称/ID 查找父亲；若不可用，尝试按默认名称
            parent_name = None
            # 尝试使用输入对象的 parent 字段（原始列表中可能存的是 id，非 name）
            # 简化处理：按常见名称匹配
            if '配液瓶(小)板' in name_to_prelim:
                parent_name = '配液瓶(小)板'
            if parent_name and parent_name in name_to_prelim:
                it['parent_name'] = parent_name
                name_to_prelim[parent_name].setdefault('children', []).append({**it})
            continue
        result.append(it)

    return result


def main():
    if not os.path.exists(BIOYOND_INPUT):
        print(f'❌ 未找到输入: {BIOYOND_INPUT}')
        return
    with open(BIOYOND_INPUT, 'r', encoding='utf-8') as f:
        bioyond_list = json.load(f)

    plr_list = bioyond_to_plr(bioyond_list)

    with open(PLR_OUTPUT, 'w', encoding='utf-8') as f:
        json.dump(plr_list, f, ensure_ascii=False, indent=2)
    print(f'✅ 已生成: {PLR_OUTPUT}')


if __name__ == '__main__':
    main()
