import importlib
import inspect
import json
from typing import Union, Any, Dict
try:
    import numpy as np
except ImportError:
    np = None
try:
    import networkx as nx
except ImportError:
    nx = None
try:
    from unilabos_msgs.msg import Resource
except Exception:
    Resource = None

try:
    from unilabos.resources.container import RegularContainer
    from unilabos.ros.msgs.message_converter import convert_to_ros_msg
except Exception:
    RegularContainer = None
    convert_to_ros_msg = None

try:
    from pylabrobot.resources.resource import Resource as ResourcePLR
except ImportError:
    pass
from typing import Union, get_origin

physical_setup_graph = None


def canonicalize_nodes_data(data: dict, parent_relation: dict = {}) -> dict:
    for node in data.get("nodes", []):
        if node.get("label") is not None:
            id = node.pop("label")
            node["id"] = node["name"] = id
        if "id" not in node:
            node["id"] = node.get("name", "NaN")
        if "name" not in node:
            node["name"] = node["id"]
        if node.get("position") is None:
            node["position"] = {
                "x": node.pop("x", 0.0),
                "y": node.pop("y", 0.0),
                "z": node.pop("z", 0.0),
            }
        if node.get("config") is None:
            node["config"] = {}
            node["data"] = {}
            for k in list(node.keys()):
                if k not in [
                    "id",
                    "name",
                    "class",
                    "type",
                    "position",
                    "children",
                    "parent",
                    "config",
                    "data",
                ]:
                    if k in ["chemical", "current_volume"]:
                        if node["data"].get("liquids") is None:
                            node["data"]["liquids"] = [{}]
                    if k == "chemical":
                        node["data"]["liquids"][0]["liquid_name"] = node.pop(k)
                    elif k == "current_volume":
                        node["data"]["liquids"][0]["liquid_volume"] = node.pop(k)
                    elif k == "max_volume":
                        node["data"]["max_volume"] = node.pop(k)
                    elif k == "url":
                        node.pop(k)
                    else:
                        node["config"][k] = node.pop(k)
        if "class" not in node:
            node["class"] = None
        if "type" not in node:
            node["type"] = (
                "container"
                if node["class"] is None
                else "device" if node["class"] not in ["container", "plate"] else node["class"]
            )
        if "children" not in node:
            node["children"] = []

    id2idx = {node_data["id"]: idx for idx, node_data in enumerate(data["nodes"])}
    for parent, children in parent_relation.items():
        data["nodes"][id2idx[parent]]["children"] = children
        for child in children:
            data["nodes"][id2idx[child]]["parent"] = parent
    return data


def canonicalize_links_ports(data: dict) -> dict:
    # 第一遍处理：将字符串类型的port转换为字典格式
    for link in data.get("links", []):
        port = link.get("port")
        if link.get("type", "physical") == "physical":
            link["type"] = "fluid"
        if isinstance(port, int):
            port = str(port)
        if isinstance(port, str):
            port_str = port.strip()
            if port_str.startswith("(") and port_str.endswith(")"):
                # 处理格式为 "(A,B)" 的情况
                content = port_str[1:-1].strip()
                parts = [p.strip() for p in content.split(",", 1)]
                source_port = parts[0]
                dest_port = parts[1] if len(parts) > 1 else None
            else:
                # 处理格式为 "A" 的情况
                source_port = port_str
                dest_port = None
            link["port"] = {link["source"]: source_port, link["target"]: dest_port}
        elif not isinstance(port, dict):
            # 若port既非字符串也非字典，初始化为空结构
            link["port"] = {link["source"]: None, link["target"]: None}

    # 构建边字典，键为(source节点, target节点)，值为对应的port信息
    edges = {(link["source"], link["target"]): link["port"] for link in data.get("links", [])}

    # 第二遍处理：填充反向边的dest信息
    delete_reverses = []
    for i, link in enumerate(data.get("links", [])):
        s, t = link["source"], link["target"]
        current_port = link["port"]
        if current_port.get(t) is None:
            reverse_key = (t, s)
            reverse_port = edges.get(reverse_key)
            if reverse_port:
                reverse_source = reverse_port.get(s)
                if reverse_source is not None:
                    # 设置当前边的dest为反向边的source
                    current_port[t] = reverse_source
                    delete_reverses.append(i)
            else:
                # 若不存在反向边，初始化为空结构
                current_port[t] = current_port[s]
    # 删除已被使用反向端口信息的反向边
    data["links"] = [link for i, link in enumerate(data.get("links", [])) if i not in delete_reverses]

    return data


def handle_communications(G):
    available_communication_types = ["serial", "io_device", "plc", "io"]
    for e, edata in G.edges.items():
        if edata.get("type", "physical") != "communication":
            continue
        if G.nodes[e[0]].get("class") in available_communication_types:
            device_comm, device = e[0], e[1]
        elif G.nodes[e[1]].get("class") in available_communication_types:
            device_comm, device = e[1], e[0]
        else:
            continue

        if G.nodes[device_comm].get("class") == "serial":
            G.nodes[device]["config"]["port"] = device_comm
        elif G.nodes[device_comm].get("class") == "io_device":
            print(f'!!! Modify {device}\'s io_device_port to {edata["port"][device_comm]}')
            G.nodes[device]["config"]["io_device_port"] = int(edata["port"][device_comm])


def read_node_link_json(json_info: Union[str, Dict[str, Any]]) -> tuple[Any, dict]:
    global physical_setup_graph
    if isinstance(json_info, str):
        data = json.load(open(json_info, encoding="utf-8"))
    else:
        data = json_info
    data = canonicalize_nodes_data(data)
    data = canonicalize_links_ports(data)

    physical_setup_graph = nx.node_link_graph(data, multigraph=False)  # edges="links" 3.6 warning
    handle_communications(physical_setup_graph)
    return physical_setup_graph, data


def modify_to_backend_format(data: list[dict[str, Any]]) -> list[dict[str, Any]]:
    for edge in data:
        port = edge.pop("port", {})
        source = edge["source"]
        target = edge["target"]
        if source in port:
            edge["sourceHandle"] = port[source]
        elif "source_port" in edge:
            edge["sourceHandle"] = edge.pop("source_port")
        if target in port:
            edge["targetHandle"] = port[target]
        elif "target_port" in edge:
            edge["targetHandle"] = edge.pop("target_port")
        edge["id"] = f"reactflow__edge-{source}-{edge['sourceHandle']}-{target}-{edge['targetHandle']}"
        for key in ["source_port", "target_port"]:
            if key in edge:
                edge.pop(key)
    return data


def read_graphml(graphml_file):
    global physical_setup_graph

    G = nx.read_graphml(graphml_file)
    mapping = {}
    parent_relation = {}
    for node in G.nodes():
        label = G.nodes[node].pop("label", G.nodes[node].get("id", G.nodes[node].get("name", "NaN")))
        mapping[node] = label
        if "::" in node:
            parent = mapping[node.split("::")[0]]
            if parent not in parent_relation:
                parent_relation[parent] = []
            parent_relation[parent].append(label)

    G2 = nx.relabel_nodes(G, mapping)
    data = nx.node_link_data(G2)
    data = canonicalize_nodes_data(data, parent_relation=parent_relation)
    data = canonicalize_links_ports(data)

    physical_setup_graph = nx.node_link_graph(data, edges="links", multigraph=False)  # edges="links" 3.6 warning
    handle_communications(physical_setup_graph)
    return physical_setup_graph, data


def dict_from_graph(graph) -> dict:
    nodes_copy = {node_id: {"id": node_id, **node} for node_id, node in graph.nodes(data=True)}
    return nodes_copy


def dict_to_tree(nodes: dict, devices_only: bool = False) -> list[dict]:
    # 将节点转换为字典，以便通过 ID 快速查找
    nodes_list = [node for node in nodes.values() if node.get("type") == "device" or not devices_only]
    id_list = [node["id"] for node in nodes_list]
    is_root = {node["id"]: True for node in nodes_list}

    # 初始化每个节点的 children 为包含节点字典的列表
    for node in nodes_list:
        node["children"] = [nodes[child_id] for child_id in node.get("children", [])]
        for child_id in node.get("children", []):
            if child_id in is_root:
                is_root[child_id] = False

    # 找到根节点并返回
    root_nodes = [
        node
        for node in nodes_list
        if is_root.get(node["id"], False) or len(nodes_list) == 1
    ]

    # 如果存在多个根节点，返回所有根节点
    return root_nodes


def dict_to_nested_dict(nodes: dict, devices_only: bool = False) -> dict:
    # 将节点转换为字典，以便通过 ID 快速查找
    nodes_list = [node for node in nodes.values() if node.get("type") == "device" or not devices_only]
    is_root = {node["id"]: True for node in nodes_list}

    # 初始化每个节点的 children 为包含节点字典的列表
    for node in nodes_list:
        node["children"] = {
            child_id: nodes[child_id]
            for child_id in node.get("children", [])
            if nodes[child_id].get("type") == "device" or not devices_only
        }
        for child_id in node.get("children", []):
            if child_id in is_root:
                is_root[child_id] = False
        if len(node["children"]) > 0 and node["type"].lower() == "device":
            node["config"]["children"] = node["children"]

    # 找到根节点并返回
    root_nodes = {
        node["id"]: node
        for node in nodes_list
        if is_root.get(node["id"], False) or len(nodes_list) == 1
    }

    # 如果存在多个根节点，返回所有根节点
    return root_nodes


def list_to_nested_dict(nodes: list[dict]) -> dict:
    nodes_dict = {node["id"]: node for node in nodes}
    return dict_to_nested_dict(nodes_dict)


def tree_to_list(tree: list[dict]) -> list[dict]:
    def _tree_to_list(tree: list[dict], result: list[dict]):
        for node_ in tree:
            node = node_.copy()
            result.append(node)
            if node.get("children"):
                _tree_to_list(node["children"], result)
            node["children"] = [n["id"] for n in node["children"]]

    result = []
    _tree_to_list(tree, result)
    return result


def nested_dict_to_list(nested_dict: dict) -> list[dict]:  # FIXME 是tree？
    """
    将嵌套字典转换为扁平列表

    嵌套字典的层次结构将通过children属性表示

    Args:
        nested_dict: 嵌套的字典结构

    Returns:
        扁平化的字典列表
    """
    result = []

    # 如果输入本身是一个节点，先添加它
    if "id" in nested_dict:
        node = nested_dict.copy()
        # 暂存子节点
        children_dict = node.get("children", {})
        # 如果children是字典，将其转换为键列表
        if isinstance(children_dict, dict):
            node["children"] = list(children_dict.keys())
        elif not isinstance(children_dict, list):
            node["children"] = []
        result.append(node)

        # 处理子节点字典
        if isinstance(children_dict, dict):
            for child_id, child_data in children_dict.items():
                if isinstance(child_data, dict):
                    # 为子节点添加ID（如果不存在）
                    if "id" not in child_data:
                        child_data["id"] = child_id
                    # 递归处理子节点
                    result.extend(nested_dict_to_list(child_data))

    # 处理children字段
    elif "children" in nested_dict:
        children_dict = nested_dict.get("children", {})
        if isinstance(children_dict, dict):
            for child_id, child_data in children_dict.items():
                if isinstance(child_data, dict):
                    # 为子节点添加ID（如果不存在）
                    if "id" not in child_data:
                        child_data["id"] = child_id
                    # 递归处理子节点
                    result.extend(nested_dict_to_list(child_data))

    return result

def convert_resources_to_type(
    resources_list: list[dict], resource_type: Union[type, list[type]], *, plr_model: bool = False
) -> Union[list[dict], dict, None, "ResourcePLR"]:
    """
    Convert resources to a given type (PyLabRobot or NestedDict) from flattened list of dictionaries.

    Args:
        resources: List of resources in the flattened dictionary format.
        resource_type: Type of the resources to convert to.
        plr_model: 是否有plr_model类型

    Returns:
        List of resources in the given type.
    """
    if resource_type == dict or resource_type == str:
        return list_to_nested_dict(resources_list)
    elif isinstance(resource_type, type) and issubclass(resource_type, ResourcePLR):
        if isinstance(resources_list, dict):
            return resource_ulab_to_plr(resources_list, plr_model)
        resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
        return resource_ulab_to_plr(resources_tree[0], plr_model)
    elif isinstance(resource_type, list):
        if all((get_origin(t) is Union) for t in resource_type):
            resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
            return [resource_ulab_to_plr(r, plr_model) for r in resources_tree]
        elif all(issubclass(t, ResourcePLR) for t in resource_type):
            resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
            return [resource_ulab_to_plr(r, plr_model) for r in resources_tree]
    # 处理 typing._GenericAlias 类型，如 List[PLRResource]
    elif hasattr(resource_type, '__origin__') and get_origin(resource_type) is list:
        # 检查泛型参数是否是 ResourcePLR 的子类
        args = getattr(resource_type, '__args__', ())
        if args and len(args) == 1 and issubclass(args[0], ResourcePLR):
            resources_tree = dict_to_tree({r["id"]: r for r in resources_list})
            return [resource_ulab_to_plr(r, plr_model) for r in resources_tree]
    else:
        return None


def convert_resources_from_type(resources_list, resource_type: Union[type, list[type]], *, is_plr: bool = False) -> Union[list[dict], dict, None, "ResourcePLR"]:
    """
    Convert resources from a given type (PyLabRobot or NestedDict) to flattened list of dictionaries.

    Args:
        resources_list: List of resources in the given type.
        resource_type: Type of the resources to convert from.

    Returns:
        List of resources in the flattened dictionary format.
    """
    if resource_type == dict:
        return nested_dict_to_list(resources_list)
    elif isinstance(resource_type, type) and issubclass(resource_type, ResourcePLR):
        resources_tree = [resource_plr_to_ulab(resources_list)]
        return tree_to_list(resources_tree)
    elif isinstance(resource_type, list):
        if all((get_origin(t) is Union) for t in resource_type):
            resources_tree = [resource_plr_to_ulab(r) for r in resources_list]
            return tree_to_list(resources_tree)
        elif is_plr or all(issubclass(t, ResourcePLR) for t in resource_type):
            resources_tree = [resource_plr_to_ulab(r) for r in resources_list]
            return tree_to_list(resources_tree)
    else:
        return None


def get_coordinates_from_bioyond_wuliao(resource_name: str, resource_class: str) -> dict:
    """
    从 Bioyond_wuliao.py 中获取资源的坐标信息
    """
    # 导入 Bioyond_wuliao 模块
    try:
        from unilabos.devices.workstation.Benyao.Bioyond_wuliao import (
            PreparationDeck, StackCarrier4x4, BottleRack, TipBox64
        )
        
        # 根据资源类型和名称返回相应的坐标
        if "枪头盒" in resource_class or "tip" in resource_name.lower():
            # 枪头盒的坐标映射
            tip_coordinates = {
                "123": {"x": 100, "y": 200, "z": 0},  # 粉末堆栈区域
                "4565": {"x": 100, "y": 295, "z": 0},  # 粉末堆栈区域
                "枪头盒3": {"x": 100, "y": 390, "z": 0},  # 粉末堆栈区域
                "枪头盒4": {"x": 100, "y": 485, "z": 0},  # 粉末堆栈区域
            }
            return tip_coordinates.get(resource_name, {"x": 0, "y": 0, "z": 0})
            
        elif "配液瓶" in resource_class or "配液瓶" in resource_name:
            # 配液瓶的坐标映射
            solution_coordinates = {
                "配液瓶(小)板": {"x": 800, "y": 200, "z": 0},  # 溶液堆栈区域
                "配液瓶(小)": {"x": 800, "y": 200, "z": 0},  # 溶液堆栈区域
            }
            return solution_coordinates.get(resource_name, {"x": 800, "y": 200, "z": 0})
            
        elif "适配器" in resource_class or "适配器" in resource_name:
            # 适配器的坐标映射
            adapter_coordinates = {
                "适配器块": {"x": 1500, "y": 200, "z": 0},  # 试剂堆栈区域
            }
            return adapter_coordinates.get(resource_name, {"x": 1500, "y": 200, "z": 0})
            
        else:
            # 默认坐标
            return {"x": 0, "y": 0, "z": 0}
            
    except ImportError:
        # 如果无法导入，返回默认坐标
        return {"x": 0, "y": 0, "z": 0}


def get_sizes_from_bioyond_wuliao(resource_name: str, resource_class: str) -> dict:
    """
    从 Bioyond_wuliao.py 中获取资源的尺寸信息
    """
    try:
        from unilabos.devices.workstation.Benyao.Bioyond_wuliao import (
            PreparationDeck, StackCarrier4x4, BottleRack, TipBox64
        )
        
        # 根据资源类型和名称返回相应的尺寸
        if "枪头盒" in resource_class or "tip" in resource_name.lower():
            # 枪头盒的尺寸映射 (基于 TipBox64)
            tip_sizes = {
                "123": {"size_x": 127.8, "size_y": 85.5, "size_z": 60.0},
                "4565": {"size_x": 127.8, "size_y": 85.5, "size_z": 60.0},
                "枪头盒3": {"size_x": 127.8, "size_y": 85.5, "size_z": 60.0},
                "枪头盒4": {"size_x": 127.8, "size_y": 85.5, "size_z": 60.0},
            }
            return tip_sizes.get(resource_name, {"size_x": 127.8, "size_y": 85.5, "size_z": 60.0})
            
        elif "配液瓶" in resource_class or "配液瓶" in resource_name:
            # 配液瓶的尺寸映射 (基于 Bottle，对于 Well 类型需要 size_x == size_y)
            solution_sizes = {
                "配液瓶(小)板": {"size_x": 13, "size_y": 13, "size_z": 11},  # 瓶子尺寸，确保 x=y
                "配液瓶(小)": {"size_x": 13, "size_y": 13, "size_z": 11},  # 瓶子尺寸，确保 x=y
            }
            return solution_sizes.get(resource_name, {"size_x": 13, "size_y": 13, "size_z": 11})
            
        elif "适配器" in resource_class or "适配器" in resource_name:
            # 适配器的尺寸映射 (基于 BottleRack)
            adapter_sizes = {
                "适配器块": {"size_x": 18.0, "size_y": 12.0, "size_z": 15.0},
            }
            return adapter_sizes.get(resource_name, {"size_x": 18.0, "size_y": 12.0, "size_z": 15.0})
            
        else:
            # 默认尺寸
            return {"size_x": 10.0, "size_y": 10.0, "size_z": 10.0}
            
    except ImportError:
        # 如果无法导入，返回默认尺寸
        return {"size_x": 10.0, "size_y": 10.0, "size_z": 10.0}


def resource_ulab_to_plr(resource: dict, plr_model=False) -> "ResourcePLR":
    """
    Resource有model字段，但是Deck下没有，这个plr由外面判断传入
    """
    if ResourcePLR is None:
        raise ImportError("pylabrobot not found")

    all_states = {resource["id"]: resource["data"]}
    # 记录所有资源的位置信息，键为资源名称（与PLR中的name一致）
    all_positions: dict[str, dict] = {}

    def resource_ulab_to_plr_inner(resource: dict):
        all_states[resource["name"]] = resource["data"]
        # 记录位置（优先使用 Bioyond_wuliao.py 中的坐标，否则使用原始 position）
        try:
            # 首先尝试从 Bioyond_wuliao.py 获取坐标
            bioyond_pos = get_coordinates_from_bioyond_wuliao(
                resource["name"], 
                resource.get("class", "")
            )
            
            # 如果 Bioyond_wuliao.py 中有坐标定义，使用它
            if bioyond_pos and bioyond_pos != {"x": 0, "y": 0, "z": 0}:
                all_positions[resource["name"]] = bioyond_pos
            else:
                # 否则使用原始 position
                pos = resource.get("position") or {}
                if isinstance(pos, dict) and {"x", "y", "z"}.issubset(pos.keys()):
                    all_positions[resource["name"]] = {"x": pos.get("x", 0), "y": pos.get("y", 0), "z": pos.get("z", 0)}
        except Exception:
            pass
        
        # 类型映射：将 unilab 类型映射到 PyLabRobot 类型
        type_mapping = {
            "container": "Container",
            "plate": "Plate", 
            "well": "Well",
            "tip_spot": "TipSpot",
            "trash": "Trash",
            "deck": "Deck",
            "tip_rack": "TipRack",
            "electrode_sheet": "Container",
            "material_hole": "Container",
            "material_plate": "Plate",
            "plate_slot": "Container",
            "coin_cell_deck": "Deck",
        }
        
        plr_type = type_mapping.get(resource["type"], "Container")  # 默认为 Container
        
        # 获取尺寸信息（优先使用 Bioyond_wuliao.py 中的尺寸，否则使用原始 config）
        bioyond_sizes = get_sizes_from_bioyond_wuliao(
            resource["name"], 
            resource.get("class", "")
        )
        
        # 过滤掉 PyLabRobot 不支持的配置参数
        plr_supported_config = {}
        plr_supported_keys = {
            "size_x", "size_y", "size_z", "model", "category", "location", "rotation",
            "name", "type", "children", "parent_name"
        }
        
        for key, value in resource["config"].items():
            if key in plr_supported_keys:
                plr_supported_config[key] = value
        
        d = {
            "name": resource["name"],
            "type": plr_type,
            "size_x": bioyond_sizes.get("size_x", resource["config"].get("size_x", 0)),
            "size_y": bioyond_sizes.get("size_y", resource["config"].get("size_y", 0)),
            "size_z": bioyond_sizes.get("size_z", resource["config"].get("size_z", 0)),
            "location": {**resource["position"], "type": "Coordinate"},
            "rotation": {"x": 0, "y": 0, "z": 0, "type": "Rotation"},  # Resource如果没有rotation，是plr版本太低
            "category": plr_type,
            "model": resource["config"].get("model", None),  # resource中deck没有model
            "children": (
                [resource_ulab_to_plr_inner(child) for child in resource["children"]]
                if isinstance(resource["children"], list)
                else [resource_ulab_to_plr_inner(child) for child_id, child in resource["children"].items()]
            ),
            # 如果原始数据没有父级，为了在序列化时保留位置信息，这里设置一个占位父级名称
            # 这样PLR在序列化根资源时也会输出location字段
            "parent_name": resource["parent"] if resource["parent"] is not None else "__ulab_export_root__",
        }
        if not plr_model:
            d.pop("model")
        return d

    d = resource_ulab_to_plr_inner(resource)
    """无法通过Resource进行反序列化，例如TipSpot必须内部序列化好，直接用TipSpot序列化会多参数，导致出错"""
    from pylabrobot.utils.object_parsing import find_subclass
    sub_cls = find_subclass(d["type"], ResourcePLR)
    spect = inspect.signature(sub_cls)
    if "category" not in spect.parameters:
        d.pop("category")
    resource_plr = sub_cls.deserialize(d, allow_marshal=True)
    resource_plr.load_all_state(all_states)

    # 反序列化后显式设置位置信息，避免根资源 location 为 None 的情况
    try:
        from pylabrobot.resources.coordinate import Coordinate

        def apply_positions(plr_res):
            if hasattr(plr_res, "name") and plr_res.name in all_positions:
                pos = all_positions[plr_res.name]
                try:
                    # 强制设置坐标
                    plr_res.location = Coordinate(x=pos.get("x", 0), y=pos.get("y", 0), z=pos.get("z", 0))
                except Exception as e:
                    # 忽略无法设置的位置
                    pass
            # 递归对子资源应用
            if getattr(plr_res, "children", None):
                for child in plr_res.children:
                    apply_positions(child)

        apply_positions(resource_plr)
    except Exception:
        # 坐标设置失败不影响主流程
        pass
    return resource_plr


def resource_plr_to_ulab(resource_plr: "ResourcePLR", parent_name: str = None, with_children=True):
    def replace_plr_type_to_ulab(source: str):
        replace_info = {
            "plate": "plate",
            "well": "well",
            "tip_spot": "container",
            "trash": "container",
            "deck": "deck",
            "tip_rack": "container",
            "electrode_sheet": "container",
            "material_hole": "container",
            "material_plate": "plate",
            "plate_slot": "container",
            "coin_cell_deck": "deck",
        }
        if source in replace_info:
            return replace_info[source]
        else:
            print("转换pylabrobot的时候，出现未知类型", source)
            return "container"
    def resource_plr_to_ulab_inner(d: dict, all_states: dict, child=True) -> dict:
        r = {
            "id": d["name"],
            "name": d["name"],
            "sample_id": None,
            "children": [resource_plr_to_ulab_inner(child, all_states) for child in d["children"]] if child else [],
            "parent": d["parent_name"] if d["parent_name"] else parent_name if parent_name else None,
            "type": replace_plr_type_to_ulab(d.get("category")),  # FIXME plr自带的type是python class name
            "class": d.get("class", ""),
            "position": (
                {"x": d["location"]["x"], "y": d["location"]["y"], "z": d["location"]["z"]}
                if d["location"]
                else {"x": 0, "y": 0, "z": 0}
            ),
            "config": {k: v for k, v in d.items() if k not in ["name", "children", "parent_name", "location"]},
            "data": all_states[d["name"]],
        }
        return r
    d = resource_plr.serialize()
    all_states = resource_plr.serialize_all_state()
    r = resource_plr_to_ulab_inner(d, all_states, with_children)

    return r


def initialize_resource(resource_config: dict) -> list[dict]:
    """Initializes a resource based on its configuration.

    If the config is detailed, then do nothing;
    If it is a string, then import the appropriate class and create an instance of it.

    Args:
        resource_config (dict): The configuration dictionary for the resource, which includes the class type and other parameters.

    Returns:
        None
    """
    from unilabos.registry.registry import lab_registry
    resource_class_config = resource_config.get("class", None)
    if resource_class_config is None:
        return [resource_config]
    elif type(resource_class_config) == str:
        # Allow special resource class names to be used
        if resource_class_config not in lab_registry.resource_type_registry:
            return [resource_config]
        # If the resource class is a string, look up the class in the
        # resource_type_registry and import it
        resource_class_config = resource_config["class"] = lab_registry.resource_type_registry[resource_class_config][
            "class"
        ]
    if type(resource_class_config) == dict:
        module = importlib.import_module(resource_class_config["module"].split(":")[0])
        mclass = resource_class_config["module"].split(":")[1]
        RESOURCE = getattr(module, mclass)

        if resource_class_config["type"] == "pylabrobot":
            resource_plr = RESOURCE(name=resource_config["name"])
            r = resource_plr_to_ulab(resource_plr=resource_plr, parent_name=resource_config.get("parent", None))
            # r = resource_plr_to_ulab(resource_plr=resource_plr)
            if resource_config.get("position") is not None:
                r["position"] = resource_config["position"]
            r = tree_to_list([r])
        elif resource_class_config["type"] == "unilabos":
            res_instance: RegularContainer = RESOURCE(id=resource_config["name"])
            res_instance.ulr_resource = convert_to_ros_msg(Resource, {k:v for k,v in resource_config.items() if k != "class"})
            r = [res_instance.get_ulr_resource_as_dict()]
        elif isinstance(RESOURCE, dict):
            r = [RESOURCE.copy()]

    return r

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
            "type": "container",  # 物料默认为容器类型
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
                    "type": "well",  # 子物料通常是孔位类型
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


def resource_bioyond_container_to_ulab(bioyond_data: dict) -> list[dict]:
    """
    将Bioyond容器格式（如bioyond_test.json）转换为UniLab格式
    这是专门处理原始bioyond_test.json数据结构的函数
    
    Args:
        bioyond_data: Bioyond格式的数据，包含data数组
        
    Returns:
        UniLab格式的资源列表
    """
    unilab_resources = []
    
    for container in bioyond_data.get("data", []):
        # 创建容器资源
        container_resource = {
            "id": container["id"],
            "name": container["name"],
            "sample_id": container.get("code"),
            "children": [],
            "parent": None,
            "type": "plate",  # 根据typeName判断，这里默认为plate
            "class": container.get("typeName", "plate"),
            "position": {"x": 0, "y": 0, "z": 0},  # 容器本身的位置
            "config": {
                "barCode": container.get("barCode", ""),
                "unit": container.get("unit", ""),
                "status": container.get("status", 0),
                "size_x": 127.76,  # 标准96孔板尺寸
                "size_y": 85.48,
                "size_z": 14.35,
                "model": "Generic 96 Well Plate",
                "num_items_x": 12,
                "num_items_y": 8,
                "well_size_x": 9.0,
                "well_size_y": 9.0,
                "well_size_z": 10.0,
                "well_bottom_type": "flat",
                "category": "plate",
                "max_volume": container.get("quantity", 0),
                "available_quantity": container.get("quantity", 0) - container.get("lockQuantity", 0)
            },
            "data": {
                "quantity": container.get("quantity", 0),
                "lockQuantity": container.get("lockQuantity", 0),
                "liquids": [],
                "pending_liquids": [],
                "liquid_history": []
            }
        }
        
        # 处理容器内的材料详情
        for detail in container.get("containerDetails", []):
            # 创建well资源
            well_resource = {
                "id": detail["id"],
                "name": f"{container['name']}_well_{detail['x']}_{detail['y']}",
                "sample_id": detail.get("code"),
                "children": [],
                "parent": container["id"],
                "type": "well",
                "class": "well",
                "position": {
                    "x": detail["x"] * 9.0 - 54.0,  # 转换为实际坐标
                    "y": detail["y"] * 9.0 - 36.0,
                    "z": detail["z"] * 10.0
                },
                "config": {
                    "detailMaterialId": detail.get("detailMaterialId"),
                    "unit": detail.get("unit", ""),
                    "associateId": detail.get("associateId"),
                    "size_x": 9.0,
                    "size_y": 9.0,
                    "size_z": 10.0,
                    "model": "Generic 96 Well",
                    "category": "well",
                    "max_volume": detail.get("quantity", 0),
                    "available_quantity": detail.get("quantity", 0) - detail.get("lockQuantity", 0)
                },
                "data": {
                    "quantity": detail.get("quantity", 0),
                    "lockQuantity": detail.get("lockQuantity", 0),
                    "liquids": [[detail["name"], detail.get("quantity", 0)]] if detail.get("name") and detail.get("quantity", 0) > 0 else [],
                    "pending_liquids": [],
                    "liquid_history": []
                }
            }
            
            # 添加材料信息到well的data中
            if detail.get("detailMaterial"):
                material = detail["detailMaterial"]
                well_resource["data"]["material_info"] = {
                    "material_id": material["id"],
                    "code": material["code"],
                    "name": material["name"],
                    "type_id": material["typeId"],
                    "parameters": material.get("parameters"),
                    "cas": material.get("cas", ""),
                    "batch_code": material.get("batchCode", ""),
                    "manufacturer": material.get("manufacturer"),
                    "preparer": material.get("preparer"),
                    "barCode": material.get("barCode", ""),
                    "status": material.get("status", 0)
                }
            
            container_resource["children"].append(well_resource["id"])
            unilab_resources.append(well_resource)
        
        unilab_resources.append(container_resource)
    
    return unilab_resources


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




def initialize_resources(resources_config) -> list[dict]:
    """Initializes a list of resources based on their configuration.

    If the config is detailed, then do nothing;
    If it is a string, then import the appropriate class and create an instance of it.

    Args:
        resources_config (list[dict]): The configuration dictionary for the resources, which includes the class type and other parameters.

    Returns:
        None
    """

    resources = []
    for resource_config in resources_config:
        resources.extend(initialize_resource(resource_config))

    return resources
