"""
API模块

提供API路由和处理函数
"""

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import asyncio

from unilabos.app.controler import devices, job_add, job_info
from unilabos.app.model import (
    Resp,
    RespCode,
    JobStatusResp,
    JobAddResp,
    JobAddReq,
    JobStepFinishReq,
    JobPreintakeFinishReq,
    JobFinishReq,
)
from unilabos.app.web.utils.host_utils import get_host_node_info

# 创建API路由器
api = APIRouter()
admin = APIRouter()

# 存储所有活动的WebSocket连接
active_connections: set[WebSocket] = set()
# 存储注册表编辑器的WebSocket连接
registry_editor_connections: set[WebSocket] = set()
# 存储状态页面的WebSocket连接
status_page_connections: set[WebSocket] = set()

# 状态跟踪变量，用于差异检测
_static_data_sent_connections: set[WebSocket] = set()
_previous_host_node_info: dict = {}
_previous_local_devices: dict = {}


def compute_host_node_diff(current: dict, previous: dict) -> dict:
    """计算主机节点信息的差异，只返回有变化的部分"""
    diff = {}

    # 检查可用性变化
    if current.get("available") != previous.get("available"):
        diff["available"] = current.get("available")

    # 检查设备列表变化
    current_devices = current.get("devices", {})
    previous_devices = previous.get("devices", {})
    if current_devices != previous_devices:
        diff["devices"] = current_devices

    # 检查动作客户端变化
    current_action_clients = current.get("action_clients", {})
    previous_action_clients = previous.get("action_clients", {})
    if current_action_clients != previous_action_clients:
        diff["action_clients"] = current_action_clients

    # 检查订阅主题变化
    current_topics = current.get("subscribed_topics", [])
    previous_topics = previous.get("subscribed_topics", [])
    if current_topics != previous_topics:
        diff["subscribed_topics"] = current_topics

    # 设备状态始终包含（因为需要实时更新）
    if "device_status" in current:
        diff["device_status"] = current["device_status"]
        diff["device_status_timestamps"] = current.get("device_status_timestamps", {})

    return diff


async def broadcast_device_status():
    """广播设备状态到所有连接的客户端"""
    while True:
        try:
            # 获取最新的设备状态
            host_info = get_host_node_info()
            if host_info["available"]:
                # 准备要发送的数据
                status_data = {
                    "type": "device_status",
                    "data": {
                        "device_status": host_info["device_status"],
                        "device_status_timestamps": host_info["device_status_timestamps"],
                    },
                }
                # 发送到所有连接的客户端
                for connection in active_connections:
                    try:
                        await connection.send_json(status_data)
                    except Exception as e:
                        print(f"Error sending to client: {e}")
                        active_connections.remove(connection)
            await asyncio.sleep(1)  # 每秒更新一次
        except Exception as e:
            print(f"Error in broadcast: {e}")
            await asyncio.sleep(1)


async def broadcast_status_page_data():
    """广播状态页面数据到所有连接的客户端（优化版：增量更新）"""
    global _previous_local_devices, _static_data_sent_connections, _previous_host_node_info

    while True:
        try:
            if status_page_connections:
                from unilabos.app.web.utils.host_utils import get_host_node_info
                from unilabos.app.web.utils.ros_utils import get_ros_node_info
                from unilabos.app.web.utils.device_utils import get_registry_info
                from unilabos.config.config import BasicConfig
                from unilabos.registry.registry import lab_registry
                from unilabos.ros.msgs.message_converter import msg_converter_manager
                from unilabos.utils.type_check import TypeEncoder
                import json

                # 获取当前数据
                host_node_info = get_host_node_info()
                ros_node_info = get_ros_node_info()

                # 检查需要发送静态数据的新连接
                new_connections = status_page_connections - _static_data_sent_connections

                # 向新连接发送静态数据（Device Types、Resource Types、Converter Modules）
                if new_connections:
                    devices = []
                    resources = []
                    modules = {"names": [], "classes": [], "displayed_count": 0, "total_count": 0}

                    if lab_registry:
                        devices = json.loads(
                            json.dumps(lab_registry.obtain_registry_device_info(), ensure_ascii=False, cls=TypeEncoder)
                        )
                        # 资源类型
                        for resource_id, resource_info in lab_registry.resource_type_registry.items():
                            resources.append(
                                {
                                    "id": resource_id,
                                    "name": resource_info.get("name", "未命名"),
                                    "file_path": resource_info.get("file_path", ""),
                                }
                            )

                    # 获取导入的模块
                    if msg_converter_manager:
                        modules["names"] = msg_converter_manager.list_modules()
                        all_classes = [i for i in msg_converter_manager.list_classes() if "." in i]
                        modules["total_count"] = len(all_classes)
                        modules["classes"] = all_classes

                    # 静态数据
                    registry_info = get_registry_info()
                    static_data = {
                        "type": "static_data_init",
                        "data": {
                            "devices": devices,
                            "resources": resources,
                            "modules": modules,
                            "registry_info": registry_info,
                            "is_host_mode": BasicConfig.is_host_mode,
                            "host_node_info": host_node_info,  # 添加主机节点初始信息
                            "ros_node_info": ros_node_info,  # 添加本地设备初始信息
                        },
                    }

                    # 发送到新连接
                    disconnected_new_connections = set()
                    for connection in new_connections:
                        try:
                            await connection.send_json(static_data)
                            _static_data_sent_connections.add(connection)
                        except Exception as e:
                            print(f"Error sending static data to new client: {e}")
                            disconnected_new_connections.add(connection)

                    # 清理断开的新连接
                    for conn in disconnected_new_connections:
                        status_page_connections.discard(conn)
                        _static_data_sent_connections.discard(conn)

                # 检查主机节点信息是否有变更
                host_node_diff = compute_host_node_diff(host_node_info, _previous_host_node_info)
                host_changed = bool(host_node_diff)

                # 检查Local Devices是否有变更
                current_devices = ros_node_info.get("registered_devices", {})
                devices_changed = current_devices != _previous_local_devices

                # 只有当有真正的变化时才发送更新
                if host_changed or devices_changed:
                    # 发送增量更新数据
                    update_data = {
                        "type": "incremental_update",
                        "data": {
                            "timestamp": asyncio.get_event_loop().time(),
                        },
                    }

                    # 只包含有变化的主机节点信息
                    if host_changed:
                        update_data["data"]["host_node_info"] = host_node_diff

                    # 如果Local Devices发生变更，添加到更新数据中
                    if devices_changed:
                        update_data["data"]["ros_node_info"] = ros_node_info
                        _previous_local_devices = current_devices.copy()

                    # 更新主机节点状态
                    if host_changed:
                        _previous_host_node_info = host_node_info.copy()

                    # 发送增量更新到所有连接
                    disconnected_connections = set()
                    for connection in status_page_connections:
                        try:
                            await connection.send_json(update_data)
                        except Exception as e:
                            print(f"Error sending incremental update to client: {e}")
                            disconnected_connections.add(connection)

                    # 清理断开的连接
                    for conn in disconnected_connections:
                        status_page_connections.discard(conn)
                        _static_data_sent_connections.discard(conn)

            await asyncio.sleep(1)  # 每秒检查一次更新
        except Exception as e:
            print(f"Error in status page broadcast: {e}")
            await asyncio.sleep(1)


@api.websocket("/ws/device_status")
async def websocket_device_status(websocket: WebSocket):
    """WebSocket端点，用于实时获取设备状态"""
    await websocket.accept()
    active_connections.add(websocket)
    try:
        while True:
            # 保持连接活跃
            await websocket.receive_text()
    except WebSocketDisconnect:
        active_connections.remove(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        active_connections.remove(websocket)


@api.websocket("/ws/registry_editor")
async def websocket_registry_editor(websocket: WebSocket):
    """WebSocket端点，用于注册表编辑器"""
    await websocket.accept()
    registry_editor_connections.add(websocket)

    try:
        while True:
            # 接收来自客户端的消息
            message = await websocket.receive_text()
            import json

            data = json.loads(message)

            if data.get("type") == "import_file":
                await handle_file_import(websocket, data["data"])
            elif data.get("type") == "analyze_file":
                await handle_file_analysis(websocket, data["data"])
            elif data.get("type") == "analyze_file_content":
                await handle_file_content_analysis(websocket, data["data"])
            elif data.get("type") == "import_file_content":
                await handle_file_content_import(websocket, data["data"])

    except WebSocketDisconnect:
        registry_editor_connections.remove(websocket)
    except Exception as e:
        print(f"Registry Editor WebSocket error: {e}")
        if websocket in registry_editor_connections:
            registry_editor_connections.remove(websocket)


@api.websocket("/ws/status_page")
async def websocket_status_page(websocket: WebSocket):
    """WebSocket端点，用于状态页面实时数据更新"""
    await websocket.accept()
    status_page_connections.add(websocket)

    try:
        while True:
            # 接收来自客户端的消息（用于保持连接活跃）
            message = await websocket.receive_text()
            # 状态页面通常只需要接收数据，不需要发送复杂指令

    except WebSocketDisconnect:
        status_page_connections.remove(websocket)
    except Exception as e:
        print(f"Status Page WebSocket error: {e}")
        if websocket in status_page_connections:
            status_page_connections.remove(websocket)


async def handle_file_analysis(websocket: WebSocket, request_data: dict):
    """处理文件分析请求，获取文件中的类列表"""
    import json
    import os
    import sys
    import inspect
    import traceback
    from pathlib import Path
    from unilabos.config.config import BasicConfig

    file_path = request_data.get("file_path")

    async def send_log(message: str, level: str = "info"):
        """发送日志消息到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "log", "message": message, "level": level}))
        except Exception as e:
            print(f"Failed to send log: {e}")

    async def send_analysis_result(result_data: dict):
        """发送分析结果到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "file_analysis_result", "data": result_data}))
        except Exception as e:
            print(f"Failed to send analysis result: {e}")

    try:
        # 验证文件路径参数
        if not file_path:
            await send_analysis_result({"success": False, "error": "文件路径为空", "file_path": ""})
            return

        # 获取工作目录并构建完整路径
        working_dir_str = getattr(BasicConfig, "working_dir", None) or os.getcwd()
        working_dir = Path(working_dir_str)
        full_file_path = working_dir / file_path

        # 验证文件路径
        if not full_file_path.exists():
            await send_analysis_result(
                {"success": False, "error": f"文件路径不存在: {file_path}", "file_path": file_path}
            )
            return

        await send_log(f"开始分析文件: {file_path}")

        # 验证文件是Python文件
        if not file_path.endswith(".py"):
            await send_analysis_result({"success": False, "error": "请选择Python文件 (.py)", "file_path": file_path})
            return

        full_file_path = full_file_path.absolute()
        await send_log(f"文件绝对路径: {full_file_path}")

        # 添加文件目录到sys.path
        file_dir = str(full_file_path.parent)
        if file_dir not in sys.path:
            sys.path.insert(0, file_dir)
            await send_log(f"已添加路径到sys.path: {file_dir}")

        # 确定模块名
        module_name = full_file_path.stem
        await send_log(f"使用模块名: {module_name}")

        # 导入模块进行分析
        try:
            # 如果模块已经导入，先删除以便重新导入
            if module_name in sys.modules:
                del sys.modules[module_name]
                await send_log(f"已删除旧模块: {module_name}")

            import importlib.util

            spec = importlib.util.spec_from_file_location(module_name, full_file_path)
            if spec is None or spec.loader is None:
                await send_analysis_result(
                    {"success": False, "error": "无法创建模块规范", "file_path": str(full_file_path)}
                )
                return

            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)

            await send_log(f"成功导入模块用于分析: {module_name}")

        except Exception as e:
            await send_analysis_result(
                {"success": False, "error": f"导入模块失败: {str(e)}", "file_path": str(full_file_path)}
            )
            return

        # 分析模块中的类
        classes = []
        for name in dir(module):
            try:
                obj = getattr(module, name)
                if isinstance(obj, type) and obj.__module__ == module_name:
                    # 获取类的文档字符串
                    docstring = inspect.getdoc(obj) or ""
                    # 只取第一行作为简短描述
                    short_desc = docstring.split("\n")[0] if docstring else ""

                    classes.append({"name": name, "docstring": short_desc, "full_docstring": docstring})
            except Exception as e:
                await send_log(f"分析类 {name} 时出错: {str(e)}", "warning")
                continue

        if not classes:
            await send_analysis_result(
                {
                    "success": False,
                    "error": "模块中未找到任何类定义",
                    "file_path": str(full_file_path),
                    "module_name": module_name,
                }
            )
            return

        await send_log(f"找到 {len(classes)} 个类: {[cls['name'] for cls in classes]}")

        # 发送分析结果
        await send_analysis_result(
            {"success": True, "file_path": str(full_file_path), "module_name": module_name, "classes": classes}
        )

    except Exception as e:
        await send_analysis_result(
            {
                "success": False,
                "error": f"分析过程中发生错误: {str(e)}",
                "file_path": file_path,
                "traceback": traceback.format_exc(),
            }
        )


async def handle_file_content_analysis(websocket: WebSocket, request_data: dict):
    """处理文件内容分析请求，直接分析上传的文件内容"""
    import json
    import os
    import sys
    import inspect
    import traceback
    import tempfile
    from pathlib import Path

    file_name = request_data.get("file_name")
    file_content = request_data.get("file_content")
    file_size = request_data.get("file_size", 0)

    async def send_log(message: str, level: str = "info"):
        """发送日志消息到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "log", "message": message, "level": level}))
        except Exception as e:
            print(f"Failed to send log: {e}")

    async def send_analysis_result(result_data: dict):
        """发送分析结果到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "file_analysis_result", "data": result_data}))
        except Exception as e:
            print(f"Failed to send analysis result: {e}")

    try:
        # 验证文件内容
        if not file_name or not file_content:
            await send_analysis_result({"success": False, "error": "文件名或文件内容为空", "file_name": file_name})
            return

        await send_log(f"开始分析文件: {file_name} ({file_size} 字节)")

        # 验证文件是Python文件
        if not file_name.endswith(".py"):
            await send_analysis_result({"success": False, "error": "请选择Python文件 (.py)", "file_name": file_name})
            return

        # 创建临时文件
        with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False, encoding="utf-8") as temp_file:
            temp_file.write(file_content)
            temp_file_path = temp_file.name

        await send_log(f"创建临时文件: {temp_file_path}")

        try:
            # 添加临时文件目录到sys.path
            temp_dir = str(Path(temp_file_path).parent)
            if temp_dir not in sys.path:
                sys.path.insert(0, temp_dir)
                await send_log(f"已添加临时目录到sys.path: {temp_dir}")

            # 确定模块名（去掉.py扩展名）
            module_name = file_name.replace(".py", "").replace("-", "_").replace(" ", "_")
            await send_log(f"使用模块名: {module_name}")

            # 导入模块进行分析
            try:
                # 如果模块已经导入，先删除以便重新导入
                if module_name in sys.modules:
                    del sys.modules[module_name]
                    await send_log(f"已删除旧模块: {module_name}")

                import importlib.util

                spec = importlib.util.spec_from_file_location(module_name, temp_file_path)
                if spec is None or spec.loader is None:
                    raise Exception("无法创建模块规范")

                module = importlib.util.module_from_spec(spec)
                sys.modules[module_name] = module
                spec.loader.exec_module(module)

                await send_log(f"成功导入模块用于分析: {module_name}")

            except Exception as e:
                await send_analysis_result(
                    {"success": False, "error": f"导入模块失败: {str(e)}", "file_name": file_name}
                )
                return

            # 分析模块中的类
            classes = []
            for name in dir(module):
                try:
                    obj = getattr(module, name)
                    if isinstance(obj, type) and obj.__module__ == module_name:
                        # 获取类的文档字符串
                        docstring = inspect.getdoc(obj) or ""
                        # 只取第一行作为简短描述
                        short_desc = docstring.split("\n")[0] if docstring else "无描述"

                        classes.append({"name": name, "docstring": short_desc, "full_docstring": docstring})
                except Exception as e:
                    await send_log(f"分析类 {name} 时出错: {str(e)}", "warning")
                    continue

            if not classes:
                await send_analysis_result(
                    {
                        "success": False,
                        "error": "模块中未找到任何类定义",
                        "file_name": file_name,
                        "module_name": module_name,
                    }
                )
                return

            await send_log(f"找到 {len(classes)} 个类: {[cls['name'] for cls in classes]}")

            # 发送分析结果
            await send_analysis_result(
                {
                    "success": True,
                    "file_name": file_name,
                    "module_name": module_name,
                    "classes": classes,
                    "temp_file_path": temp_file_path,  # 保存临时文件路径供后续使用
                }
            )

        finally:
            # 清理临时文件（在导入完成后再删除）
            try:
                if os.path.exists(temp_file_path):
                    # 延迟删除，给导入操作留出时间
                    import threading

                    def delayed_cleanup():
                        import time

                        time.sleep(60)  # 等待60秒后删除
                        try:
                            os.unlink(temp_file_path)
                        except OSError:
                            pass

                    threading.Thread(target=delayed_cleanup, daemon=True).start()
            except Exception as e:
                await send_log(f"清理临时文件时出错: {str(e)}", "warning")

    except Exception as e:
        await send_analysis_result(
            {
                "success": False,
                "error": f"分析过程中发生错误: {str(e)}",
                "file_name": file_name,
                "traceback": traceback.format_exc(),
            }
        )


async def handle_file_content_import(websocket: WebSocket, request_data: dict):
    """处理基于文件内容的导入请求"""
    import json
    import os
    import sys
    import traceback
    import tempfile
    from pathlib import Path

    file_name = request_data.get("file_name")
    file_content = request_data.get("file_content")
    file_size = request_data.get("file_size", 0)
    registry_type = request_data.get("registry_type", "device")
    class_name = request_data.get("class_name")

    async def send_log(message: str, level: str = "info"):
        """发送日志消息到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "log", "message": message, "level": level}))
        except Exception as e:
            print(f"Failed to send log: {e}")

    async def send_progress(message: str):
        """发送进度消息到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "progress", "message": message}))
        except Exception as e:
            print(f"Failed to send progress: {e}")

    async def send_error(message: str):
        """发送错误消息到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "error", "message": message}))
        except Exception as e:
            print(f"Failed to send error: {e}")

    async def send_result(result_data: dict):
        """发送结果数据到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "result", "data": result_data}))
        except Exception as e:
            print(f"Failed to send result: {e}")

    try:
        # 验证输入参数
        if not file_name or not file_content or not class_name:
            await send_error("文件名、文件内容或类名为空")
            return

        await send_log(f"开始处理文件: {file_name} ({file_size} 字节)")
        await send_progress("正在创建临时文件...")

        # 创建临时文件
        with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False, encoding="utf-8") as temp_file:
            temp_file.write(file_content)
            temp_file_path = temp_file.name

        await send_log(f"创建临时文件: {temp_file_path}")

        # 添加临时文件目录到sys.path
        temp_dir = str(Path(temp_file_path).parent)
        if temp_dir not in sys.path:
            sys.path.insert(0, temp_dir)
            await send_log(f"已添加临时目录到sys.path: {temp_dir}")

        # 确定模块名
        module_name = file_name.replace(".py", "").replace("-", "_").replace(" ", "_")
        await send_log(f"使用模块名: {module_name}")

        # 导入模块
        try:
            # 如果模块已经导入，先删除以便重新导入
            if module_name in sys.modules:
                del sys.modules[module_name]
                await send_log(f"已删除旧模块: {module_name}")

            import importlib.util

            spec = importlib.util.spec_from_file_location(module_name, temp_file_path)
            if spec is None or spec.loader is None:
                await send_error("无法创建模块规范")
                return

            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)

            await send_log(f"成功导入模块: {module_name}")

        except Exception as e:
            await send_error(f"导入模块失败: {str(e)}")
            return

        # 验证类存在
        if not hasattr(module, class_name):
            await send_error(f"模块中未找到类: {class_name}")
            return

        target_class = getattr(module, class_name)
        await send_log(f"找到目标类: {class_name}")

        # 使用registry.py的增强类信息功能进行分析
        await send_progress("正在生成注册表信息...")

        try:
            from unilabos.utils.import_manager import get_enhanced_class_info

            # 分析类信息
            enhanced_info = get_enhanced_class_info(f"{module_name}:{class_name}", use_dynamic=True)

            if not enhanced_info.get("dynamic_import_success", False):
                await send_error("动态导入类信息失败")
                return

            await send_log("成功分析类信息")

            # 生成注册表schema
            registry_schema = {
                "class_name": class_name,
                "module": f"{module_name}:{class_name}",
                "type": "python",
                "description": enhanced_info.get("class_docstring", ""),
                "version": "1.0.0",
                "category": [registry_type],
                "status_types": {k: v["return_type"] for k, v in enhanced_info["status_methods"].items()},
                "action_value_mappings": {},
                "init_param_schema": {},
                "registry_type": registry_type,
                "file_path": f"uploaded_file://{file_name}",
            }

            # 处理动作方法
            for method_name, method_info in enhanced_info["action_methods"].items():
                registry_schema["action_value_mappings"][f"auto-{method_name}"] = {
                    "type": "UniLabJsonCommandAsync" if method_info["is_async"] else "UniLabJsonCommand",
                    "goal": {},
                    "feedback": {},
                    "result": {},
                    "args": method_info["args"],
                    "description": method_info.get("docstring", ""),
                }

            await send_log("成功生成注册表schema")

            # 准备结果数据
            result = {
                "class_info": {
                    "class_name": class_name,
                    "module_name": module_name,
                    "file_name": file_name,
                    "file_size": file_size,
                    "docstring": enhanced_info.get("class_docstring", ""),
                    "dynamic_import_success": enhanced_info.get("dynamic_import_success", False),
                },
                "registry_schema": registry_schema,
                "action_methods": enhanced_info["action_methods"],
                "status_methods": enhanced_info["status_methods"],
            }

            # 发送结果
            await send_result(result)
            await send_log("分析完成")

        except Exception as e:
            await send_error(f"分析类信息时发生错误: {str(e)}")
            await send_log(f"详细错误信息: {traceback.format_exc()}")
            return

        finally:
            # 清理临时文件
            try:
                if os.path.exists(temp_file_path):
                    import threading

                    def delayed_cleanup():
                        import time

                        time.sleep(30)  # 等待30秒后删除
                        try:
                            os.unlink(temp_file_path)
                        except OSError:
                            pass

                    threading.Thread(target=delayed_cleanup, daemon=True).start()
            except Exception as e:
                await send_log(f"清理临时文件时出错: {str(e)}", "warning")

    except Exception as e:
        await send_error(f"处理过程中发生错误: {str(e)}")
        await send_log(f"详细错误信息: {traceback.format_exc()}")


async def handle_file_import(websocket: WebSocket, request_data: dict):
    """处理文件导入请求"""
    import json
    import os
    import sys
    import traceback
    from pathlib import Path
    from unilabos.config.config import BasicConfig

    file_path = request_data.get("file_path")
    registry_type = request_data.get("registry_type", "device")
    class_name = request_data.get("class_name")
    module_name = request_data.get("module_name")

    async def send_log(message: str, level: str = "info"):
        """发送日志消息到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "log", "message": message, "level": level}))
        except Exception as e:
            print(f"Failed to send log: {e}")

    async def send_progress(message: str):
        """发送进度消息到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "progress", "message": message}))
        except Exception as e:
            print(f"Failed to send progress: {e}")

    async def send_error(message: str):
        """发送错误消息到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "error", "message": message}))
        except Exception as e:
            print(f"Failed to send error: {e}")

    async def send_result(result_data: dict):
        """发送结果数据到客户端"""
        try:
            await websocket.send_text(json.dumps({"type": "result", "data": result_data}))
        except Exception as e:
            print(f"Failed to send result: {e}")

    try:
        # 验证文件路径参数
        if not file_path:
            await send_error("文件路径为空")
            return

        # 获取工作目录并构建完整路径
        working_dir_str = getattr(BasicConfig, "working_dir", None) or os.getcwd()
        working_dir = Path(working_dir_str)
        full_file_path = working_dir / file_path

        # 验证文件路径
        if not full_file_path.exists():
            await send_error(f"文件路径不存在: {file_path}")
            return

        await send_log(f"开始处理文件: {file_path}")
        await send_progress("正在验证文件...")

        # 验证文件是Python文件
        if not file_path.endswith(".py"):
            await send_error("请选择Python文件 (.py)")
            return

        full_file_path = full_file_path.absolute()
        await send_log(f"文件绝对路径: {full_file_path}")

        # 动态导入模块
        await send_progress("正在导入模块...")

        # 添加文件目录到sys.path
        file_dir = str(full_file_path.parent)
        if file_dir not in sys.path:
            sys.path.insert(0, file_dir)
            await send_log(f"已添加路径到sys.path: {file_dir}")

        # 确定模块名
        if not module_name:
            module_name = full_file_path.stem
        await send_log(f"使用模块名: {module_name}")

        # 导入模块
        try:
            # 如果模块已经导入，先删除以便重新导入
            if module_name in sys.modules:
                del sys.modules[module_name]
                await send_log(f"已删除旧模块: {module_name}")

            import importlib.util

            spec = importlib.util.spec_from_file_location(module_name, full_file_path)
            if spec is None or spec.loader is None:
                await send_error("无法创建模块规范")
                return

            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)

            await send_log(f"成功导入模块: {module_name}")

        except Exception as e:
            await send_error(f"导入模块失败: {str(e)}")
            return

        # 分析模块
        await send_progress("正在分析模块...")

        # 获取模块中的所有类
        classes = []
        for name in dir(module):
            obj = getattr(module, name)
            if isinstance(obj, type) and obj.__module__ == module_name:
                classes.append((name, obj))

        if not classes:
            await send_error("模块中未找到任何类定义")
            return

        await send_log(f"找到 {len(classes)} 个类: {[name for name, _ in classes]}")

        # 确定要分析的类
        target_class = None
        target_class_name = None

        if class_name:
            # 用户指定了类名
            for name, cls in classes:
                if name == class_name:
                    target_class = cls
                    target_class_name = name
                    break
            if not target_class:
                await send_error(f"未找到指定的类: {class_name}")
                return
        else:
            # 自动选择第一个类
            target_class_name, target_class = classes[0]
            await send_log(f"自动选择类: {target_class_name}")

        # 使用registry.py的增强类信息功能进行分析
        await send_progress("正在生成注册表信息...")

        try:
            from unilabos.utils.import_manager import get_enhanced_class_info

            # 分析类信息
            enhanced_info = get_enhanced_class_info(f"{module_name}:{target_class_name}", use_dynamic=True)

            if not enhanced_info.get("dynamic_import_success", False):
                await send_error("动态导入类信息失败")
                return

            await send_log("成功分析类信息")

            # 生成注册表schema
            registry_schema = {
                "class_name": target_class_name,
                "module": f"{module_name}:{target_class_name}",
                "type": "python",
                "description": enhanced_info.get("class_docstring", ""),
                "version": "1.0.0",
                "category": [registry_type],
                "status_types": {k: v["return_type"] for k, v in enhanced_info["status_methods"].items()},
                "action_value_mappings": {},
                "init_param_schema": {},
                "registry_type": registry_type,
                "file_path": str(full_file_path),
            }

            # 处理动作方法
            for method_name, method_info in enhanced_info["action_methods"].items():
                registry_schema["action_value_mappings"][f"auto-{method_name}"] = {
                    "type": "UniLabJsonCommandAsync" if method_info["is_async"] else "UniLabJsonCommand",
                    "goal": {},
                    "feedback": {},
                    "result": {},
                    "args": method_info["args"],
                    "description": method_info.get("docstring", ""),
                }

            await send_log("成功生成注册表schema")

            # 转换为YAML格式
            import yaml
            from unilabos.utils.type_check import NoAliasDumper

            # 创建最终的YAML配置（使用设备ID作为根键）
            class_name_safe = class_name or "unknown"
            suffix = "_device" if registry_type == "device" else "_resource"
            device_id = f"{class_name_safe.lower()}{suffix}"
            final_config = {device_id: registry_schema}

            yaml_content = yaml.dump(
                final_config, allow_unicode=True, default_flow_style=False, Dumper=NoAliasDumper, sort_keys=True
            )

            # 准备结果数据（只保留YAML结果）
            result = {
                "registry_schema": yaml_content,
                "device_id": device_id,
                "class_name": class_name,
                "module_name": module_name,
                "file_path": file_path,
            }

            # 发送结果
            await send_result(result)
            await send_log("注册表生成完成")

        except Exception as e:
            await send_error(f"分析类信息时发生错误: {str(e)}")
            await send_log(f"详细错误信息: {traceback.format_exc()}")
            return

    except Exception as e:
        await send_error(f"处理过程中发生错误: {str(e)}")
        await send_log(f"详细错误信息: {traceback.format_exc()}")


@api.get("/file-browser", summary="Browse files and directories", response_model=Resp)
def get_file_browser_data(path: str = ""):
    """获取文件浏览器数据"""
    import os
    from pathlib import Path
    from unilabos.config.config import BasicConfig

    try:
        # 获取工作目录
        working_dir_str = getattr(BasicConfig, "working_dir", None) or os.getcwd()
        working_dir = Path(working_dir_str)

        # 如果提供了相对路径，则在工作目录下查找
        if path:
            target_path = working_dir / path
        else:
            target_path = working_dir

        # 确保路径在工作目录内（安全检查）
        target_path = target_path.resolve()

        if not target_path.exists():
            return Resp(code=RespCode.ErrorInvalidReq, message=f"路径不存在: {path}")

        if not target_path.is_dir():
            return Resp(code=RespCode.ErrorInvalidReq, message=f"不是目录: {path}")

        # 获取目录内容
        items = []

        parent_path = target_path.parent
        relative_parent = parent_path.relative_to(working_dir)
        items.append(
            {
                "name": "..",
                "type": "directory",
                "path": str(relative_parent) if relative_parent != Path(".") else "",
                "size": 0,
                "is_parent": True,
            }
        )

        # 获取子目录和文件
        try:
            for item in sorted(target_path.iterdir(), key=lambda x: (not x.is_dir(), x.name.lower())):
                if item.name.startswith("."):  # 跳过隐藏文件
                    continue

                item_type = "directory" if item.is_dir() else "file"
                relative_path = item.relative_to(working_dir)

                item_info = {
                    "name": item.name,
                    "type": item_type,
                    "path": str(relative_path),
                    "size": item.stat().st_size if item.is_file() else 0,
                    "is_python": item.suffix == ".py" if item.is_file() else False,
                    "is_parent": False,
                }
                items.append(item_info)
        except PermissionError:
            return Resp(code=RespCode.ErrorInvalidReq, message="无权限访问此目录")

        return Resp(
            data={
                "current_path": str(target_path.relative_to(working_dir)) if target_path != working_dir else "",
                "working_dir": str(working_dir),
                "items": items,
            }
        )

    except Exception as e:
        return Resp(code=RespCode.ErrorInvalidReq, message=f"获取目录信息失败: {str(e)}")


@api.get("/resources", summary="Resource list", response_model=Resp)
def get_resources():
    """获取资源列表"""
    isok, data = devices()
    if not isok:
        return Resp(code=RespCode.ErrorHostNotInit, message=str(data))

    return Resp(data=dict(data))


@api.get("/devices", summary="Device list", response_model=Resp)
def get_devices():
    """获取设备列表"""
    isok, data = devices()
    if not isok:
        return Resp(code=RespCode.ErrorHostNotInit, message=str(data))

    return Resp(data=dict(data))


@api.get("/job/{id}/status", summary="Job status", response_model=JobStatusResp)
def job_status(id: str):
    """获取任务状态"""
    data = job_info(id)
    return JobStatusResp(data=data)


@api.post("/job/add", summary="Create job", response_model=JobAddResp)
def post_job_add(req: JobAddReq):
    """创建任务"""
    device_id = req.device_id
    if not req.data:
        return Resp(code=RespCode.ErrorInvalidReq, message="Invalid request data")

    req.device_id = device_id
    data = job_add(req)
    return JobAddResp(data=data)


def setup_api_routes(app):
    """设置API路由"""
    app.include_router(admin, prefix="/admin/v1", tags=["admin"])
    app.include_router(api, prefix="/api/v1", tags=["api"])

    # 启动广播任务
    @app.on_event("startup")
    async def startup_event():
        asyncio.create_task(broadcast_device_status())
        asyncio.create_task(broadcast_status_page_data())
