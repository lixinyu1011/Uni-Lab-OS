"""
HTTP客户端模块

提供与远程服务器通信的客户端功能，只有host需要用
"""

import json
import os
from typing import List, Dict, Any, Optional

import requests
from unilabos.utils.log import info
from unilabos.config.config import HTTPConfig, BasicConfig
from unilabos.utils import logger


class HTTPClient:
    """HTTP客户端，用于与远程服务器通信"""

    def __init__(self, remote_addr: Optional[str] = None, auth: Optional[str] = None) -> None:
        """
        初始化HTTP客户端

        Args:
            remote_addr: 远程服务器地址，如果不提供则从配置中获取
            auth: 授权信息
        """
        self.remote_addr = remote_addr or HTTPConfig.remote_addr
        if auth is not None:
            self.auth = auth
        else:
            auth_secret = BasicConfig.auth_secret()
            self.auth = auth_secret
            info(f"正在使用ak sk作为授权信息：[{auth_secret}]")
        info(f"HTTPClient 初始化完成: remote_addr={self.remote_addr}")

    def resource_edge_add(self, resources: List[Dict[str, Any]]) -> requests.Response:
        """
        添加资源

        Args:
            resources: 要添加的资源列表
            database_process_later: 后台处理资源
        Returns:
            Response: API响应对象
        """
        response = requests.post(
            f"{self.remote_addr}/lab/material/edge",
            json={
                "edges": resources,
            },
            headers={"Authorization": f"Lab {self.auth}"},
            timeout=100,
        )
        if response.status_code == 200:
            res = response.json()
            if "code" in res and res["code"] != 0:
                logger.error(f"添加物料关系失败: {response.text}")
        if response.status_code != 200 and response.status_code != 201:
            logger.error(f"添加物料关系失败: {response.status_code}, {response.text}")
        return response

    def resource_add(self, resources: List[Dict[str, Any]]) -> requests.Response:
        """
        添加资源

        Args:
            resources: 要添加的资源列表
            database_process_later: 后台处理资源
        Returns:
            Response: API响应对象
        """
        response = requests.post(
            f"{self.remote_addr}/lab/material",
            json={"nodes": resources},
            headers={"Authorization": f"Lab {self.auth}"},
            timeout=100,
        )
        if response.status_code == 200:
            res = response.json()
            if "code" in res and res["code"] != 0:
                logger.error(f"添加物料失败: {response.text}")
        if response.status_code != 200:
            logger.error(f"添加物料失败: {response.text}")
        return response

    def resource_get(self, id: str, with_children: bool = False) -> Dict[str, Any]:
        """
        获取资源

        Args:
            id: 资源ID
            with_children: 是否包含子资源

        Returns:
            Dict: 返回的资源数据
        """
        response = requests.get(
            f"{self.remote_addr}/lab/material",
            params={"id": id, "with_children": with_children},
            headers={"Authorization": f"Lab {self.auth}"},
            timeout=20,
        )
        return response.json()

    def resource_del(self, id: str) -> requests.Response:
        """
        删除资源

        Args:
            id: 要删除的资源ID

        Returns:
            Response: API响应对象
        """
        response = requests.delete(
            f"{self.remote_addr}/lab/resource/batch_delete/",
            params={"id": id},
            headers={"Authorization": f"Lab {self.auth}"},
            timeout=20,
        )
        return response

    def resource_update(self, resources: List[Dict[str, Any]]) -> requests.Response:
        """
        更新资源

        Args:
            resources: 要更新的资源列表

        Returns:
            Response: API响应对象
        """
        return self.resource_add(resources)
        response = requests.patch(
            f"{self.remote_addr}/lab/resource/batch_update/?edge_format=1",
            json=resources,
            headers={"Authorization": f"Lab {self.auth}"},
            timeout=100,
        )
        return response

    def upload_file(self, file_path: str, scene: str = "models") -> requests.Response:
        """
        上传文件到服务器

        使用multipart/form-data格式上传文件，类似curl -F "files=@filepath"

        Args:
            file_path: 要上传的文件路径
            scene: 上传场景，可选值为"user"或"models"，默认为"models"

        Returns:
            Response: API响应对象
        """
        with open(file_path, "rb") as file:
            files = {"files": file}
            logger.info(f"上传文件: {file_path} 到 {scene}")
            response = requests.post(
                f"{self.remote_addr}/api/account/file_upload/{scene}",
                files=files,
                headers={"Authorization": f"Lab {self.auth}"},
                timeout=30,  # 上传文件可能需要更长的超时时间
            )
        return response

    def resource_registry(self, registry_data: Dict[str, Any] | List[Dict[str, Any]]) -> requests.Response:
        """
        注册资源到服务器

        Args:
            registry_data: 注册表数据，格式为 {resource_id: resource_info} / [{resource_info}]

        Returns:
            Response: API响应对象
        """
        response = requests.post(
            f"{self.remote_addr}/lab/resource",
            json=registry_data,
            headers={"Authorization": f"Lab {self.auth}"},
            timeout=30,
        )
        if response.status_code not in [200, 201]:
            logger.error(f"注册资源失败: {response.status_code}, {response.text}")
        return response

    def request_startup_json(self) -> Optional[Dict[str, Any]]:
        """
        请求启动配置

        Args:
            startup_json: 启动配置JSON数据

        Returns:
            Response: API响应对象
        """
        response = requests.get(
            f"{self.remote_addr}/lab/resource/graph_info/",
            headers={"Authorization": f"Lab {self.auth}"},
            timeout=(3, 30),
        )
        if response.status_code != 200:
            logger.error(f"请求启动配置失败: {response.status_code}, {response.text}")
        else:
            try:
                with open(os.path.join(BasicConfig.working_dir, "startup_config.json"), "w", encoding="utf-8") as f:
                    f.write(response.text)
                target_dict = json.loads(response.text)
                if "data" in target_dict:
                    target_dict = target_dict["data"]
                return target_dict
            except json.JSONDecodeError as e:
                logger.error(f"解析启动配置JSON失败: {str(e.args)}\n响应内容: {response.text}")
                logger.error(f"响应内容: {response.text}")
        return None


# 创建默认客户端实例
http_client = HTTPClient()
