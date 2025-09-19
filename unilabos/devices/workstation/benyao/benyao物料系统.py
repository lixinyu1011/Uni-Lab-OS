# 瀚文部分
import json
from pathlib import Path

# 使用脚本所在目录作为基准路径
script_dir = Path(__file__).parent
json_path = script_dir / "bioyond_test_yibin2.json"
bioyond_resources = json.loads(json_path.read_text(encoding="utf-8"))


# ulab_resources = resource_ulab_to_plr(bioyond_resources_unilab)

# # 海明部分

# deck = Deck()
# for r in ulab_resources:
#   deck.assign_child_resource(r, rails=r.location.rails[0])
from unilabos.resources.graphio import resource_ulab_to_plr, convert_resources_to_type


def resource_bioyond_to_ulab(bioyond_materials: list[dict], location_id_mapping: dict = None) -> list[dict]:
    """
    将 bioyond 物料格式转换为 ulab 物料格式

    Args:
        bioyond_materials: bioyond 系统的物料查询结果列表
        location_id_mapping: 库位 ID 到名称的映射字典，格式 {location_id: location_name}

    Returns:
        ulab 格式的物料列表
    """
    ulab_materials = []

    for material in bioyond_materials:
        # 基础物料信息
        base_material = {
            "id": material["id"],
            "name": material["name"],
            "sample_id": material.get("code"),  # 使用 code 作为 sample_id
            "children": [],
            "parent": None,
            "type": "Container",  # 物料默认为容器类型
            "class": material.get("typeName", ""),
            "position": {"x": 0, "y": 0, "z": 0},  # 默认位置
            "config": {
                "barCode": material.get("barCode", ""),
                "unit": material.get("unit", ""),
                "status": material.get("status", 0),
                "max_volume": material.get("quantity", 0),
                "available_quantity": material.get("quantity", 0) - material.get("lockQuantity", 0)
            },
            "data": {
                "quantity": material.get("quantity", 0),
                "lockQuantity": material.get("lockQuantity", 0),
                "liquids": [[material["name"], material.get("quantity", 0)]] if material.get("quantity", 0) > 0 else [],
                "pending_liquids": [],
                "liquid_history": []
            }
        }

        # 设置物料位置信息（基于第一个 location）
        if material.get("locations") and len(material["locations"]) > 0:
            location = material["locations"][0]
            # 设置父级容器（仓库）
            if location_id_mapping and location.get("whid") in location_id_mapping:
                base_material["parent"] = location_id_mapping[location["whid"]]
            else:
                base_material["parent"] = location.get("whName", f"warehouse_{location.get('whid', 'unknown')}")

            # 设置位置坐标
            base_material["position"] = {
                "x": location.get("x", 0),
                "y": location.get("y", 0),
                "z": location.get("z", 0)
            }

            # 在 config 中保存库位信息
            base_material["config"]["location_info"] = {
                "location_id": location.get("id"),
                "warehouse_id": location.get("whid"),
                "warehouse_name": location.get("whName"),
                "code": location.get("code"),
                "quantity_at_location": location.get("quantity", 0)
            }

        # 处理子物料（detail）
        if material.get("detail") and len(material["detail"]) > 0:
            child_ids = []
            for detail in material["detail"]:
                child_material = {
                    "id": detail["id"],
                    "name": detail["name"],
                    "sample_id": detail.get("code"),
                    "children": [],
                    "parent": material["id"],  # 父级为当前物料
                    "type": "Well",  # 子物料通常是孔位类型
                    "class": "",
                    "position": {
                        "x": detail.get("x", 0),
                        "y": detail.get("y", 0),
                        "z": detail.get("z", 0)
                    },
                    "config": {
                        "detailMaterialId": detail.get("detailMaterialId"),
                        "unit": detail.get("unit", ""),
                        "associateId": detail.get("associateId"),
                        "max_volume": float(detail.get("quantity", 0)) if detail.get("quantity") else 0,
                        "available_quantity": (float(detail.get("quantity", 0)) - float(detail.get("lockQuantity", 0)))
                                            if detail.get("quantity") and detail.get("lockQuantity") else 0
                    },
                    "data": {
                        "quantity": float(detail.get("quantity", 0)) if detail.get("quantity") else 0,
                        "lockQuantity": float(detail.get("lockQuantity", 0)) if detail.get("lockQuantity") else 0,
                        "liquids": [[detail["name"], float(detail.get("quantity", 0))]] if detail.get("quantity") and float(detail.get("quantity", 0)) > 0 else [],
                        "pending_liquids": [],
                        "liquid_history": []
                    }
                }

                ulab_materials.append(child_material)
                child_ids.append(detail["id"])

            # 更新父物料的 children 列表
            base_material["children"] = child_ids

        ulab_materials.append(base_material)

    return ulab_materials


def resource_ulab_to_bioyond(ulab_materials: list[dict], warehouse_id_mapping: dict = None) -> list[dict]:
    """
    将 ulab 物料格式转换为 bioyond 物料格式

    Args:
        ulab_materials: ulab 格式的物料列表
        warehouse_id_mapping: 仓库名称到 ID 的映射字典，格式 {warehouse_name: warehouse_id}

    Returns:
        bioyond 格式的物料列表
    """
    # 将列表转换为字典以便查找
    materials_dict = {material["id"]: material for material in ulab_materials}

    # 找出根级物料（没有父级或父级不在当前列表中的物料）
    root_materials = []
    for material in ulab_materials:
        parent_id = material.get("parent")
        if not parent_id or parent_id not in materials_dict:
            root_materials.append(material)

    bioyond_materials = []

    for root_material in root_materials:
        # 跳过设备类型的物料
        if root_material.get("type") == "device":
            continue

        bioyond_material = {
            "id": root_material["id"],
            "typeName": root_material.get("class"),
            "code": root_material.get("sample_id", ""),
            "barCode": root_material.get("config", {}).get("barCode", ""),
            "name": root_material["name"],
            "quantity": root_material.get("data", {}).get("quantity", 0),
            "lockQuantity": root_material.get("data", {}).get("lockQuantity", 0),
            "unit": root_material.get("config", {}).get("unit", ""),
            "status": root_material.get("config", {}).get("status", 0),
            "locations": [],
            "detail": []
        }

        # 构建位置信息
        if root_material.get("parent"):
            # 从 config 中获取位置信息，如果没有则使用默认值
            location_info = root_material.get("config", {}).get("location_info", {})
            position = root_material.get("position", {})

            location = {
                "id": location_info.get("location_id", f"loc_{root_material['id']}"),
                "whid": warehouse_id_mapping.get(root_material["parent"], root_material["parent"]) if warehouse_id_mapping else root_material["parent"],
                "whName": location_info.get("warehouse_name", root_material["parent"]),
                "code": location_info.get("code", ""),
                "x": position.get("x", 0),
                "y": position.get("y", 0),
                "z": position.get("z", 0),
                "quantity": location_info.get("quantity_at_location", 0)
            }
            bioyond_material["locations"] = [location]

        # 处理子物料
        if root_material.get("children"):
            for child_id in root_material["children"]:
                if child_id in materials_dict:
                    child = materials_dict[child_id]
                    detail_item = {
                        "id": child["id"],
                        "detailMaterialId": child.get("config", {}).get("detailMaterialId", child["id"]),
                        "code": child.get("sample_id"),
                        "name": child["name"],
                        "quantity": str(child.get("data", {}).get("quantity", 0)),
                        "lockQuantity": str(child.get("data", {}).get("lockQuantity", 0)),
                        "unit": child.get("config", {}).get("unit", ""),
                        "x": child.get("position", {}).get("x", 0),
                        "y": child.get("position", {}).get("y", 0),
                        "z": child.get("position", {}).get("z", 0),
                        "associateId": child.get("config", {}).get("associateId")
                    }
                    bioyond_material["detail"].append(detail_item)

        bioyond_materials.append(bioyond_material)

    return bioyond_materials


from typing import List, Dict, Any

def remove_redundant_resources(resources_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    对资源进行去重并排序：
    - 对所有在 children 中被引用的 id：仅保留一个条目，优先保留有 parent 的版本（子节点版本）
    - 对未在 children 中被引用的 id：仅保留一个条目，优先保留 parent 为空的版本（根节点版本）
    - 清理各资源的 children 列表，仅保留仍存在的子节点 id，且去重并保持原顺序
    - 最后将资源按照 children 数量从多到少排序（children 越多越靠上；数量相同保持原相对顺序）

    Args:
        resources_list: 资源列表

    Returns:
        去重、清理并按 children 数量降序排列的资源列表
    """
    # 收集所有作为 children 的 id
    referenced_child_ids = set()
    for resource in resources_list:
        children = resource.get("children", []) or []
        referenced_child_ids.update(children)

    # 构建 id -> 该 id 的所有条目 列表
    id_to_entries: Dict[str, List[Dict[str, Any]]] = {}
    for resource in resources_list:
        rid = resource.get("id")
        if not rid:
            continue
        id_to_entries.setdefault(rid, []).append(resource)

    kept: List[Dict[str, Any]] = []

    # 1) 对被引用为 children 的 id：仅保留一个，优先保留有 parent 的版本
    for rid in referenced_child_ids:
        entries = id_to_entries.get(rid, [])
        if not entries:
            # 若出现引用但无实体，填充一个最小占位，避免后续构树报 KeyError
            kept.append({
                "id": rid,
                "name": rid,
                "children": [],
                "parent": None,
                "type": "Container",
                "class": "",
                "position": {"x": 0, "y": 0, "z": 0},
                "config": {},
                "data": {}
            })
            continue
        preferred = next((e for e in entries if e.get("parent") is not None), None)
        kept.append(preferred if preferred is not None else entries[0])

    # 2) 对未被引用为 children 的 id：仅保留一个，优先保留 parent 为空（根）的版本
    for rid, entries in id_to_entries.items():
        if rid in referenced_child_ids:
            continue
        preferred = next((e for e in entries if e.get("parent") in (None, "")), None)
        kept.append(preferred if preferred is not None else entries[0])

    # 3) 清理 children：仅保留仍存在的 id，并去重（保持原顺序）
    kept_ids = {r["id"] for r in kept if r.get("id")}
    for res in kept:
        children = (res.get("children") or [])
        if children:
            # 只保留仍存在的子节点
            filtered = [cid for cid in children if cid in kept_ids]
            # 去重并保持顺序
            seen = set()
            deduped = []
            for cid in filtered:
                if cid not in seen:
                    seen.add(cid)
                    deduped.append(cid)
            res["children"] = deduped
        else:
            res["children"] = []

    # 4) 按 children 数量从多到少排序；同数目时保持原相对顺序（稳定排序）
    # 为保持稳定性，先记录当前顺序索引
    original_index = {id(res): idx for idx, res in enumerate(kept)}
    kept.sort(key=lambda r: (-len(r.get("children") or []), original_index[id(r)]))

    return kept



    # print(bioyond_resources)

bioyond_resources_unilab = resource_bioyond_to_ulab(bioyond_resources["data"])
# 删除冗余信息
bioyond_resources_unilab = remove_redundant_resources(bioyond_resources_unilab)
# print(bioyond_resources_unilab)



from typing import List
from pylabrobot.resources import Resource as PLRResource

ulab_resources = convert_resources_to_type(bioyond_resources_unilab, [PLRResource])
# ulab_resources = convert_resources_to_type(bioyond_resources_unilab, dict)
# ulab_resources = resource_ulab_to_plr(bioyond_resources_unilab)

# ulab_resources = [resource_ulab_to_plr(r) for r in bioyond_resources_unilab]
# ulab_resources = [convert_resources_to_type([r],PLRResource) for r in bioyond_resources_unilab]

print(ulab_resources)





