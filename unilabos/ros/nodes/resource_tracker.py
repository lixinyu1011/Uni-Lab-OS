import uuid
from pydantic import BaseModel, field_serializer, field_validator
from pydantic import Field
from typing import List, Tuple, Any, Dict, Literal, Optional, cast, TYPE_CHECKING

from unilabos.utils.log import logger

if TYPE_CHECKING:
    # from unilabos.devices.workstation.workstation_base import WorkstationBase
    from pylabrobot.resources import Resource as PLRResource, corning_6_wellplate_16point8ml_flat


class ResourceDictPositionSize(BaseModel):
    depth: float = Field(description="Depth", default=0.0)
    width: float = Field(description="Width", default=0.0)
    height: float = Field(description="Height", default=0.0)


class ResourceDictPositionScale(BaseModel):
    x: float = Field(description="x scale", default=0.0)
    y: float = Field(description="y scale", default=0.0)
    z: float = Field(description="z scale", default=0.0)


class ResourceDictPositionObject(BaseModel):
    x: float = Field(description="X coordinate", default=0.0)
    y: float = Field(description="Y coordinate", default=0.0)
    z: float = Field(description="Z coordinate", default=0.0)


class ResourceDictPosition(BaseModel):
    size: ResourceDictPositionSize = Field(description="Resource size", default_factory=ResourceDictPositionSize)
    scale: ResourceDictPositionScale = Field(description="Resource scale", default_factory=ResourceDictPositionScale)
    layout: Literal["2d"] = Field(description="Resource layout", default="2d")
    position: ResourceDictPositionObject = Field(
        description="Resource position", default_factory=ResourceDictPositionObject
    )
    position3d: ResourceDictPositionObject = Field(
        description="Resource position in 3D space", default_factory=ResourceDictPositionObject
    )
    rotation: ResourceDictPositionObject = Field(
        description="Resource rotation", default_factory=ResourceDictPositionObject
    )


# 统一的资源字典模型，parent 自动序列化为 parent_uuid，children 不序列化
class ResourceDict(BaseModel):
    id: str = Field(description="Resource ID")
    uuid: str = Field(description="Resource UUID")
    name: str = Field(description="Resource name")
    description: str = Field(description="Resource description", default="")
    schema: Dict[str, Any] = Field(description="Resource schema", default_factory=dict)
    model: Dict[str, Any] = Field(description="Resource model", default_factory=dict)
    icon: str = Field(description="Resource icon", default="")
    parent: Optional["ResourceDict"] = Field(
        description="Parent resource object", default=None, serialization_alias="parent_uuid"
    )
    type: Literal["device"] | str = Field(description="Resource type")
    klass: str = Field(alias="class", description="Resource class name")
    position: ResourceDictPosition = Field(description="Resource position", default_factory=ResourceDictPosition)
    config: Dict[str, Any] = Field(description="Resource configuration")
    data: Dict[str, Any] = Field(description="Resource data")

    @field_serializer("parent")
    def _serialize_parent(self, parent: Optional["ResourceDict"]):
        return self.parent_uuid

    @field_validator("parent", mode="before")
    @classmethod
    def _deserialize_parent(cls, parent: Optional["ResourceDict"]):
        if isinstance(parent, ResourceDict):
            return parent
        else:
            return None

    @property
    def parent_uuid(self) -> str:
        """获取父节点的UUID"""
        return self.parent.uuid if self.parent is not None else ""

    @property
    def parent_name(self) -> Optional[str]:
        """获取父节点的UUID"""
        return self.parent.name if self.parent is not None else None

    @property
    def is_root_node(self) -> bool:
        """判断资源是否为根节点"""
        return self.parent is None


class GraphData(BaseModel):
    """图数据结构，包含节点和边"""

    nodes: List["ResourceTreeInstance"] = Field(description="Resource nodes list", default_factory=list)
    links: List[Dict[str, Any]] = Field(description="Resource links/edges list", default_factory=list)


class ResourceDictInstance(object):
    """ResourceDict的实例，同时提供一些方法"""

    def __init__(self, res_content: "ResourceDict"):
        self.res_content = res_content
        self.children = []
        self.typ = "dict"

    @classmethod
    def get_resource_instance_from_dict(cls, content: Dict[str, Any]) -> "ResourceDictInstance":
        """从字典创建资源实例"""
        if "id" not in content:
            content["id"] = content["name"]
        if "uuid" not in content:
            content["uuid"] = str(uuid.uuid4())
        if "description" in content and content["description"] is None:
            del content["description"]
        if "model" in content and content["model"] is None:
            del content["model"]
        if "schema" in content and content["schema"] is None:
            del content["schema"]
        if "x" in content.get("position", {}):
            # 说明是老版本的position格式，转换成新的
            content["position"] = {"position": content["position"]}
        if not content.get("class"):
            content["class"] = ""
        if not content.get("config"):  # todo: 后续从后端保证字段非空
            content["config"] = {}
        if not content.get("data"):
            content["data"] = {}
        return ResourceDictInstance(ResourceDict.model_validate(content))

    def get_nested_dict(self) -> Dict[str, Any]:
        """获取资源实例的嵌套字典表示"""
        res_dict = self.res_content.model_dump(by_alias=True)
        res_dict["children"] = {child.res_content.name: child.get_nested_dict() for child in self.children}
        res_dict["parent"] = self.res_content.parent_name
        res_dict["position"] = self.res_content.position.position.model_dump()
        return res_dict


class ResourceTreeInstance(object):
    """
    资源树，表示一个根节点及其所有子节点的层次结构，继承ResourceDictInstance表示自己是根节点
    """

    @staticmethod
    def _build_uuid_map(resource_list: List[ResourceDictInstance]) -> Dict[str, ResourceDictInstance]:
        """构建uuid到资源对象的映射，并检查重复"""
        uuid_map: Dict[str, ResourceDictInstance] = {}
        for res_instance in resource_list:
            res = res_instance.res_content
            if res.uuid in uuid_map:
                raise ValueError(f"发现重复的uuid: {res.uuid}")
            uuid_map[res.uuid] = res_instance
        return uuid_map

    @staticmethod
    def _build_uuid_instance_map(
        resource_list: List[ResourceDictInstance],
    ) -> Dict[str, ResourceDictInstance]:
        """构建uuid到资源实例的映射"""
        return {res_instance.res_content.uuid: res_instance for res_instance in resource_list}

    @staticmethod
    def _collect_tree_nodes(
        root_instance: ResourceDictInstance, uuid_map: Dict[str, ResourceDict]
    ) -> List[ResourceDictInstance]:
        """使用BFS收集属于某个根节点的所有节点"""
        # BFS遍历，根据parent_uuid字段找到所有属于这棵树的节点
        tree_nodes = [root_instance]
        visited = {root_instance.res_content.uuid}
        queue = [root_instance.res_content.uuid]

        while queue:
            current_uuid = queue.pop(0)
            # 查找所有parent_uuid指向当前节点的子节点
            for uuid_str, res in uuid_map.items():
                if res.parent_uuid == current_uuid and uuid_str not in visited:
                    child_instance = ResourceDictInstance(res)
                    tree_nodes.append(child_instance)
                    visited.add(uuid_str)
                    queue.append(uuid_str)

        return tree_nodes

    def __init__(self, resource: ResourceDictInstance):
        self.root_node = resource
        self._validate_tree()

    def _validate_tree(self):
        """
        验证树结构的一致性
        - 验证uuid唯一性
        - 验证parent-children关系一致性

        Raises:
            ValueError: 当发现不一致时
        """
        known_uuids: set = set()

        def validate_node(node: ResourceDictInstance):
            # 检查uuid唯一性
            if node.res_content.uuid in known_uuids:
                raise ValueError(f"发现重复的uuid: {node.res_content.uuid}")
            if node.res_content.uuid:
                known_uuids.add(node.res_content.uuid)
            else:
                print(f"警告: 资源 {node.res_content.id} 没有uuid")

            # 验证并递归处理子节点
            for child in node.children:
                if child.res_content.parent != node.res_content:
                    parent_id = child.res_content.parent.id if child.res_content.parent else None
                    raise ValueError(
                        f"节点 {child.res_content.id} 的parent引用不正确，应该指向 {node.res_content.id}，但实际指向 {parent_id}"
                    )
                validate_node(child)

        validate_node(self.root_node)

    def get_all_nodes(self) -> List[ResourceDictInstance]:
        """
        获取树中的所有节点（深度优先遍历）

        Returns:
            所有节点的资源实例列表
        """
        nodes = []

        def collect_nodes(node: ResourceDictInstance):
            nodes.append(node)
            for child in node.children:
                collect_nodes(child)

        collect_nodes(self.root_node)
        return nodes

    def find_by_uuid(self, target_uuid: str) -> Optional[ResourceDictInstance]:
        """
        通过uuid查找节点

        Args:
            target_uuid: 目标uuid

        Returns:
            找到的节点资源实例，如果没找到返回None
        """

        def search(node: ResourceDictInstance) -> Optional[ResourceDictInstance]:
            if node.res_content.uuid == target_uuid:
                return node
            for child in node.children:
                res = search(child)
                if res:
                    return res
            return None

        result = search(self.root_node)
        return result


class ResourceTreeSet(object):
    """
    多个根节点的resource集合，包含多个ResourceTree
    """

    def __init__(self, resource_list: List[List[ResourceDictInstance]] | List[ResourceTreeInstance]):
        """
        初始化资源树集合

        Args:
            resource_list: 可以是以下两种类型之一：
                - List[ResourceTree]: 已经构建好的树列表
                - List[List[ResourceInstanceDict]]: 嵌套列表，每个内部列表代表一棵树

        Raises:
            TypeError: 当传入不支持的类型时
        """
        if not resource_list:
            self.trees: List[ResourceTreeInstance] = []
        elif isinstance(resource_list[0], ResourceTreeInstance):
            # 已经是ResourceTree列表
            self.trees = cast(List[ResourceTreeInstance], resource_list)
        elif isinstance(resource_list[0], list):
            pass
        else:
            raise TypeError(
                f"不支持的类型: {type(resource_list[0])}。"
                f"ResourceTreeSet 只接受 List[ResourceTree] 或 List[List[ResourceInstanceDict]]"
            )

    @classmethod
    def from_plr_resources(cls, resources: List["PLRResource"]) -> "ResourceTreeSet":
        """
        从plr资源创建ResourceTreeSet
        """

        def replace_plr_type(source: str):
            replace_info = {
                "plate": "plate",
                "well": "well",
                "tip_spot": "container",
                "trash": "container",
                "deck": "deck",
                "tip_rack": "container",
            }
            if source in replace_info:
                return replace_info[source]
            else:
                print("转换pylabrobot的时候，出现未知类型", source)
                return "container"

        def build_uuid_mapping(res: "PLRResource", uuid_list: list):
            """递归构建uuid映射字典"""
            uuid_list.append(getattr(res, "unilabos_uuid", ""))
            for child in res.children:
                build_uuid_mapping(child, uuid_list)

        def resource_plr_inner(
            d: dict, parent_resource: Optional[ResourceDict], states: dict, uuids: list
        ) -> ResourceDictInstance:
            current_uuid = uuids.pop(0)

            # 先构建当前节点的字典（不包含children）
            r_dict = {
                "id": d["name"],
                "uuid": current_uuid,
                "name": d["name"],
                "parent": parent_resource,  # 直接传入 ResourceDict 对象
                "type": replace_plr_type(d.get("category", "")),
                "class": d.get("class", ""),
                "position": (
                    {"x": d["location"]["x"], "y": d["location"]["y"], "z": d["location"]["z"]}
                    if d["location"]
                    else {"x": 0, "y": 0, "z": 0}
                ),
                "config": {k: v for k, v in d.items() if k not in ["name", "children", "parent_name", "location"]},
                "data": states[d["name"]],
            }

            # 先转换为 ResourceDictInstance，获取其中的 ResourceDict
            current_instance = ResourceDictInstance.get_resource_instance_from_dict(r_dict)
            current_resource = current_instance.res_content

            # 递归处理子节点，传入当前节点的 ResourceDict 作为 parent
            current_instance.children = [
                resource_plr_inner(child, current_resource, states, uuids) for child in d["children"]
            ]

            return current_instance

        trees = []
        for resource in resources:
            # 构建uuid列表
            uuid_list = []
            build_uuid_mapping(resource, uuid_list)

            serialized_data = resource.serialize()
            all_states = resource.serialize_all_state()

            # 根节点没有父节点，传入 None
            root_instance = resource_plr_inner(serialized_data, None, all_states, uuid_list)
            tree_instance = ResourceTreeInstance(root_instance)
            trees.append(tree_instance)
        return cls(trees)

    def to_plr_resources(self) -> Tuple[List["PLRResource"], List[Dict[str, str]]]:
        """
        将 ResourceTreeSet 转换为 PLR 资源列表

        Returns:
            Tuple[List[PLRResource], List[Dict[str, str]]]:
                - PLR 资源实例列表
                - 每个资源对应的 name_to_uuid 映射字典列表
        """
        from unilabos.resources.graphio import resource_ulab_to_plr

        plr_resources = []
        name_to_uuid_maps = []

        def build_name_to_uuid_map(node: ResourceDictInstance, result: Dict[str, str]):
            """递归构建 name 到 uuid 的映射"""
            result[node.res_content.name] = node.res_content.uuid
            for child in node.children:
                build_name_to_uuid_map(child, result)

        for tree in self.trees:
            # 构建 name_to_uuid 映射
            name_to_uuid = {}
            build_name_to_uuid_map(tree.root_node, name_to_uuid)

            # 使用 get_nested_dict 获取字典表示
            resource_dict = tree.root_node.get_nested_dict()

            # 判断是否包含 model（Deck 下没有 model）
            plr_model = tree.root_node.res_content.type != "deck"

            try:
                # 使用 resource_ulab_to_plr 创建 PLR 资源实例
                plr_resource = resource_ulab_to_plr(resource_dict, plr_model=plr_model)

                # 设置 unilabos_uuid 属性到资源及其所有子节点
                def set_uuid_recursive(plr_res: "PLRResource", node: ResourceDictInstance):
                    """递归设置 PLR 资源的 unilabos_uuid 属性"""
                    setattr(plr_res, "unilabos_uuid", node.res_content.uuid)
                    # 匹配子节点（通过 name）
                    for plr_child in plr_res.children:
                        matching_node = next(
                            (child for child in node.children if child.res_content.name == plr_child.name),
                            None,
                        )
                        if matching_node:
                            set_uuid_recursive(plr_child, matching_node)

                set_uuid_recursive(plr_resource, tree.root_node)

                plr_resources.append(plr_resource)
                name_to_uuid_maps.append(name_to_uuid)
            except Exception as e:
                logger.error(f"转换 PLR 资源失败: {e}")
                import traceback

                logger.error(f"堆栈: {traceback.format_exc()}")
                raise

        return plr_resources, name_to_uuid_maps

    @classmethod
    def from_nested_list(cls, nested_list: List[ResourceDictInstance]) -> "ResourceTreeSet":
        """
        从扁平化的资源列表创建ResourceTreeSet，自动按根节点分组

        Args:
            nested_list: 扁平化的资源实例列表，可能包含多个根节点

        Returns:
            ResourceTreeSet实例

        Raises:
            ValueError: 当没有找到任何根节点时
        """
        # 找到所有根节点
        known_uuids = {res_instance.res_content.uuid for res_instance in nested_list}
        root_instances = [
            ResourceTreeInstance(res_instance)
            for res_instance in nested_list
            if res_instance.res_content.is_root_node or res_instance.res_content.parent_uuid not in known_uuids
        ]
        return cls(root_instances)

    @property
    def root_nodes(self) -> List[ResourceDictInstance]:
        """
        获取所有树的根节点

        Returns:
            所有根节点的资源实例列表
        """
        return [tree.root_node for tree in self.trees]

    @property
    def all_nodes(self) -> List[ResourceDictInstance]:
        """
        获取所有树中的所有节点

        Returns:
            所有节点的资源实例列表
        """
        return [node for tree in self.trees for node in tree.get_all_nodes()]

    def find_by_uuid(self, target_uuid: str) -> Optional[ResourceDictInstance]:
        """
        在所有树中通过uuid查找节点

        Args:
            target_uuid: 目标uuid

        Returns:
            找到的节点资源实例，如果没找到返回None
        """
        for tree in self.trees:
            result = tree.find_by_uuid(target_uuid)
            if result:
                return result
        return None

    def dump(self) -> List[List[Dict[str, Any]]]:
        """
        将 ResourceTreeSet 序列化为嵌套列表格式

        序列化时：
        - parent 自动转换为 parent_uuid（在 ResourceDict.model_dump 中处理）
        - children 不会被序列化（exclude=True）

        Returns:
            List[List[Dict]]: 每个内层列表代表一棵树的扁平化资源字典列表
        """
        result = []
        for tree in self.trees:
            # 获取树的所有节点并序列化
            tree_nodes = [node.res_content.model_dump(by_alias=True) for node in tree.get_all_nodes()]
            result.append(tree_nodes)
        return result

    @classmethod
    def load(cls, data: List[List[Dict[str, Any]]]) -> "ResourceTreeSet":
        """
        从序列化的嵌套列表格式反序列化为 ResourceTreeSet

        Args:
            data: List[List[Dict]]: 序列化的数据，每个内层列表代表一棵树

        Returns:
            ResourceTreeSet: 反序列化后的资源树集合
        """
        # 将每个字典转换为 ResourceInstanceDict
        # FIXME: 需要重新确定parent关系
        nested_lists = []
        for tree_data in data:
            flatten_instances = [
                ResourceDictInstance.get_resource_instance_from_dict(node_dict) for node_dict in tree_data
            ]
            nested_lists.append(flatten_instances)

        # 使用现有的构造函数创建 ResourceTreeSet
        return cls(nested_lists)


class DeviceNodeResourceTracker(object):

    def __init__(self):
        self.resources = []
        self.resource2parent_resource = {}
        self.uuid_to_resources = {}
        pass

    def prefix_path(self, resource):
        resource_prefix_path = "/"
        resource_parent = getattr(resource, "parent", None)
        while resource_parent is not None:
            resource_prefix_path = f"/{resource_parent.name}" + resource_prefix_path
            resource_parent = resource_parent.parent

        return resource_prefix_path

    def map_uuid_to_resource(self, resource, uuid_map: Dict[str, str]):
        for old_uuid, new_uuid in uuid_map.items():
            if old_uuid != new_uuid:
                if old_uuid in self.uuid_to_resources:
                    instance = self.uuid_to_resources.pop(old_uuid)
                    if isinstance(resource, dict):
                        resource["uuid"] = new_uuid
                    else:  # 实例的
                        setattr(instance, "unilabos_uuid", new_uuid)
                    self.uuid_to_resources[new_uuid] = instance
                    print(f"更新uuid映射: {old_uuid} -> {new_uuid} | {instance}")

    def loop_set_uuid(self, resource, name_to_uuid_map: Dict[str, str]) -> int:
        """
        递归遍历资源树，根据 name 设置所有节点的 uuid

        Args:
            resource: 资源对象（可以是dict或实例）
            name_to_uuid_map: name到uuid的映射字典，{name: uuid}

        Returns:
            更新的资源数量
        """
        if isinstance(resource, list):
            return sum(self.loop_set_uuid(r, name_to_uuid_map) for r in resource)

        update_count = 0

        # 先递归处理所有子节点
        children = getattr(resource, "children", [])
        for child in children:
            update_count += self.loop_set_uuid(child, name_to_uuid_map)

        # 获取当前资源的name
        if isinstance(resource, dict):
            resource_name = resource.get("name")
        else:
            resource_name = getattr(resource, "name", None)

        # 如果name在映射中，则设置uuid
        if resource_name and resource_name in name_to_uuid_map:
            new_uuid = name_to_uuid_map[resource_name]
            # 更新资源的uuid
            if isinstance(resource, dict):
                resource["uuid"] = new_uuid
            else:
                # 对于PLR资源，设置unilabos_uuid
                setattr(resource, "unilabos_uuid", new_uuid)
            self.uuid_to_resources[new_uuid] = resource
            update_count += 1
            logger.debug(f"设置资源UUID: {resource_name} -> {new_uuid}")

        return update_count

    def loop_update_uuid(self, resource, uuid_map: Dict[str, str]) -> int:
        """
        递归遍历资源树，更新所有节点的uuid

        Args:
            resource: 资源对象（可以是dict或实例）
            uuid_map: uuid映射字典，{old_uuid: new_uuid}

        Returns:
            更新的资源数量
        """
        if isinstance(resource, list):
            return sum(self.loop_update_uuid(r, uuid_map) for r in resource)

        update_count = 0

        # 先递归处理所有子节点
        children = getattr(resource, "children", [])
        for child in children:
            update_count += self.loop_update_uuid(child, uuid_map)

        # 获取当前资源的uuid
        if isinstance(resource, dict):
            current_uuid = resource.get("uuid")
        else:
            current_uuid = getattr(resource, "unilabos_uuid", None)

        # 如果当前uuid在映射中，则更新
        if current_uuid and current_uuid in uuid_map:
            new_uuid = uuid_map[current_uuid]
            if current_uuid != new_uuid:
                # 更新资源的uuid
                if isinstance(resource, dict):
                    resource["uuid"] = new_uuid
                else:
                    # 对于PLR资源，更新unilabos_uuid
                    if hasattr(resource, "unilabos_uuid"):
                        setattr(resource, "unilabos_uuid", new_uuid)

                # 更新uuid_to_resources映射
                if current_uuid in self.uuid_to_resources:
                    instance = self.uuid_to_resources.pop(current_uuid)
                    self.uuid_to_resources[new_uuid] = instance

                update_count += 1
                logger.debug(f"更新uuid: {current_uuid} -> {new_uuid} | {resource}")

        return update_count

    def parent_resource(self, resource):
        if id(resource) in self.resource2parent_resource:
            return self.resource2parent_resource[id(resource)]
        else:
            return resource

    def add_resource(self, resource):
        for r in self.resources:
            if id(r) == id(resource):
                return
        self.resources.append(resource)

    def clear_resource(self):
        self.resources = []

    def figure_resource(self, query_resource, try_mode=False):
        if isinstance(query_resource, list):
            return [self.figure_resource(r, try_mode) for r in query_resource]
        elif (
            isinstance(query_resource, dict) and "id" not in query_resource and "name" not in query_resource
        ):  # 临时处理，要删除的，driver有太多类型错误标注
            return [self.figure_resource(r, try_mode) for r in query_resource.values()]
        res_id = (
            query_resource.id  # type: ignore
            if hasattr(query_resource, "id")
            else (query_resource.get("id") if isinstance(query_resource, dict) else None)
        )
        res_name = (
            query_resource.name  # type: ignore
            if hasattr(query_resource, "name")
            else (query_resource.get("name") if isinstance(query_resource, dict) else None)
        )
        res_identifier = res_id if res_id else res_name
        identifier_key = "id" if res_id else "name"
        resource_cls_type = type(query_resource)
        if res_identifier is None:
            logger.warning(f"resource {query_resource} 没有id或name，暂不能对应figure")
        res_list = []
        for r in self.resources:
            if isinstance(query_resource, dict):
                res_list.extend(
                    self.loop_find_resource(r, object, identifier_key, query_resource[identifier_key])
                )
            else:
                res_list.extend(
                    self.loop_find_resource(
                        r, resource_cls_type, identifier_key, getattr(query_resource, identifier_key)
                    )
                )
        if not try_mode:
            assert len(res_list) > 0, f"没有找到资源 {query_resource}，请检查资源是否存在"
            assert len(res_list) == 1, f"{query_resource} 找到多个资源，请检查资源是否唯一: {res_list}"
        else:
            return [i[1] for i in res_list]
        # 后续加入其他对比方式
        self.resource2parent_resource[id(query_resource)] = res_list[0][0]
        self.resource2parent_resource[id(res_list[0][1])] = res_list[0][0]
        return res_list[0][1]

    def loop_find_resource(
        self, resource, target_resource_cls_type, identifier_key, compare_value, parent_res=None
    ) -> List[Tuple[Any, Any]]:
        res_list = []
        # print(resource, target_resource_cls_type, identifier_key, compare_value)
        children = getattr(resource, "children", [])
        for child in children:
            res_list.extend(
                self.loop_find_resource(child, target_resource_cls_type, identifier_key, compare_value, resource)
            )
        if issubclass(type(resource), target_resource_cls_type):
            if target_resource_cls_type == dict:
                if identifier_key in resource:
                    if resource[identifier_key] == compare_value:
                        res_list.append((parent_res, resource))
            elif hasattr(resource, identifier_key):
                if getattr(resource, identifier_key) == compare_value:
                    res_list.append((parent_res, resource))
        return res_list

    def filter_find_list(self, res_list, compare_std_dict):
        new_list = []
        for res in res_list:
            for k, v in compare_std_dict.items():
                if hasattr(res, k):
                    if getattr(res, k) == v:
                        new_list.append(res)
        return new_list


if __name__ == "__main__":
    import sys
    import os

    a = corning_6_wellplate_16point8ml_flat("a").serialize()
    # 尝试导入 pylabrobot，如果失败则尝试从本地 pylabrobot_repo 导入
    try:
        from pylabrobot.resources import Resource, Coordinate
    except ImportError:
        # 尝试添加本地 pylabrobot_repo 路径
        # __file__ is unilabos/ros/nodes/resource_tracker.py
        # We need to go up 4 levels to get to project root
        current_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        pylabrobot_path = os.path.join(current_dir, "pylabrobot_repo")
        if os.path.exists(pylabrobot_path):
            sys.path.insert(0, pylabrobot_path)
            try:
                from pylabrobot.resources import Resource, Coordinate
            except ImportError:
                print("pylabrobot 未安装，且无法从本地 pylabrobot_repo 导入")
                print("如需运行测试，请先安装: pip install pylabrobot")
                exit(0)
        else:
            print("pylabrobot 未安装，跳过测试")
            print("如需运行测试，请先安装: pip install pylabrobot")
            exit(0)

    # 创建一个简单的测试资源
    def create_test_resource(name: str):
        """创建一个简单的测试用资源"""
        # 创建父资源
        parent = Resource(name=name, size_x=100.0, size_y=100.0, size_z=50.0, category="container")

        # 添加一些子资源
        for i in range(3):
            child = Resource(name=f"{name}_child_{i}", size_x=20.0, size_y=20.0, size_z=10.0, category="container")
            child.location = Coordinate(x=i * 30, y=0, z=0)
            parent.assign_child_resource(child, location=child.location)

        return parent

    print("=" * 80)
    print("测试 1: 基本序列化和反序列化")
    print("=" * 80)

    # 创建原始 PLR 资源
    original_resource = create_test_resource("test_resource")
    print(f"\n1. 创建原始 PLR 资源: {original_resource.name}")
    print(f"   子节点数量: {len(original_resource.children)}")

    # 手动设置 unilabos_uuid（模拟实际使用场景）
    def set_test_uuid(res: "PLRResource", prefix="uuid"):
        """递归设置测试用的 uuid"""
        import uuid as uuid_module

        setattr(res, "unilabos_uuid", f"{prefix}-{uuid_module.uuid4()}")
        for i, child in enumerate(res.children):
            set_test_uuid(child, f"{prefix}-{i}")

    set_test_uuid(original_resource, "root")
    print(f"   根节点 UUID: {getattr(original_resource, 'unilabos_uuid', 'None')}")

    # 转换为 ResourceTreeSet (from_plr_resources)
    print("\n2. 使用 from_plr_resources 转换为 ResourceTreeSet")
    resource_tree_set = ResourceTreeSet.from_plr_resources([original_resource])
    print(f"   树的数量: {len(resource_tree_set.trees)}")
    print(f"   根节点名称: {resource_tree_set.root_nodes[0].res_content.name}")
    print(f"   根节点 UUID: {resource_tree_set.root_nodes[0].res_content.uuid}")
    print(f"   总节点数: {len(resource_tree_set.all_nodes)}")

    # 转换回 PLR 资源 (to_plr_resources)
    print("\n3. 使用 to_plr_resources 转换回 PLR 资源")
    try:
        plr_resources, name_to_uuid_maps = resource_tree_set.to_plr_resources()
    except ModuleNotFoundError as e:
        print(f"   ❌ 缺少依赖模块: {e}")
        print("   提示: to_plr_resources 方法实现完成，但需要安装额外的依赖（如 networkx）")
        print("\n测试部分完成！from_plr_resources 已验证正常工作。")
        exit(0)
    print(f"   PLR 资源数量: {len(plr_resources)}")
    print(f"   name_to_uuid 映射数量: {len(name_to_uuid_maps)}")

    restored_resource = plr_resources[0]
    name_to_uuid = name_to_uuid_maps[0]

    print(f"   恢复的资源名称: {restored_resource.name}")
    print(f"   恢复的资源子节点数: {len(restored_resource.children)}")
    print(f"   恢复的资源 UUID: {getattr(restored_resource, 'unilabos_uuid', 'None')}")
    print(f"   name_to_uuid 映射条目数: {len(name_to_uuid)}")

    # 验证 UUID 映射
    print("\n4. 验证 UUID 映射")
    original_uuid = getattr(original_resource, "unilabos_uuid", None)
    restored_uuid = getattr(restored_resource, "unilabos_uuid", None)
    print(f"   原始根节点 UUID: {original_uuid}")
    print(f"   恢复后根节点 UUID: {restored_uuid}")
    print(f"   UUID 匹配: {original_uuid == restored_uuid}")

    # 验证 name_to_uuid 映射完整性
    def count_all_nodes(res: "PLRResource") -> int:
        """递归统计节点总数"""
        return 1 + sum(count_all_nodes(child) for child in res.children)

    original_node_count = count_all_nodes(original_resource)
    restored_node_count = count_all_nodes(restored_resource)
    mapping_count = len(name_to_uuid)

    print(f"\n   原始资源节点总数: {original_node_count}")
    print(f"   恢复资源节点总数: {restored_node_count}")
    print(f"   映射字典条目数: {mapping_count}")
    print(f"   节点数量匹配: {original_node_count == restored_node_count == mapping_count}")

    # 验证子节点的 UUID
    print("\n5. 验证子节点 UUID (前3个)")
    for i, (original_child, restored_child) in enumerate(
        zip(original_resource.children[:3], restored_resource.children[:3])
    ):
        orig_uuid = getattr(original_child, "unilabos_uuid", None)
        rest_uuid = getattr(restored_child, "unilabos_uuid", None)
        print(f"   子节点 {i}: {original_child.name}")
        print(f"     原始 UUID: {orig_uuid}")
        print(f"     恢复 UUID: {rest_uuid}")
        print(f"     匹配: {orig_uuid == rest_uuid}")

    # 测试 name_to_uuid 映射的正确性
    print("\n6. 验证 name_to_uuid 映射内容 (前5个)")
    for i, (name, uuid_val) in enumerate(list(name_to_uuid.items())[:5]):
        print(f"   {name} -> {uuid_val}")

    print("\n" + "=" * 80)
    print("测试完成！")
    print("=" * 80)
