import argparse
import asyncio
import os
import shutil
import signal
import sys
import threading
import time
from copy import deepcopy

import yaml


# 首先添加项目根目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
unilabos_dir = os.path.dirname(os.path.dirname(current_dir))
if unilabos_dir not in sys.path:
    sys.path.append(unilabos_dir)

from unilabos.config.config import load_config, BasicConfig, HTTPConfig
from unilabos.utils.banner_print import print_status, print_unilab_banner
from unilabos.resources.graphio import modify_to_backend_format


def load_config_from_file(config_path):
    if config_path is None:
        config_path = os.environ.get("UNILABOS_BASICCONFIG_CONFIG_PATH", None)
    if config_path:
        if not os.path.exists(config_path):
            print_status(f"配置文件 {config_path} 不存在", "error")
        elif not config_path.endswith(".py"):
            print_status(f"配置文件 {config_path} 不是Python文件，必须以.py结尾", "error")
        else:
            load_config(config_path)
    else:
        print_status(f"启动 UniLab-OS时，配置文件参数未正确传入 --config '{config_path}' 尝试本地配置...", "warning")
        load_config(config_path)


def convert_argv_dashes_to_underscores(args: argparse.ArgumentParser):
    # easier for user input, easier for dev search code
    option_strings = list(args._option_string_actions.keys())
    for i, arg in enumerate(sys.argv):
        for option_string in option_strings:
            if arg.startswith(option_string):
                new_arg = arg[:2] + arg[2 : len(option_string)].replace("-", "_") + arg[len(option_string) :]
                sys.argv[i] = new_arg
                break


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="Start Uni-Lab Edge server.")
    parser.add_argument("-g", "--graph", help="Physical setup graph file path.")
    parser.add_argument("-c", "--controllers", default=None, help="Controllers config file path.")
    parser.add_argument(
        "--registry_path",
        type=str,
        default=None,
        action="append",
        help="Path to the registry directory",
    )
    parser.add_argument(
        "--working_dir",
        type=str,
        default=None,
        help="Path to the working directory",
    )
    parser.add_argument(
        "--backend",
        choices=["ros", "simple", "automancer"],
        default="ros",
        help="Choose the backend to run with: 'ros', 'simple', or 'automancer'.",
    )
    parser.add_argument(
        "--app_bridges",
        nargs="+",
        default=["websocket", "fastapi"],
        help="Bridges to connect to. Now support 'websocket' and 'fastapi'.",
    )
    parser.add_argument(
        "--is_slave",
        action="store_true",
        help="Run the backend as slave node (without host privileges).",
    )
    parser.add_argument(
        "--slave_no_host",
        action="store_true",
        help="Skip waiting for host service in slave mode",
    )
    parser.add_argument(
        "--upload_registry",
        action="store_true",
        help="Upload registry information when starting unilab",
    )
    parser.add_argument(
        "--use_remote_resource",
        action="store_true",
        help="Use remote resources when starting unilab",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Configuration file path, supports .py format Python config files",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8002,
        help="Port for web service information page",
    )
    parser.add_argument(
        "--disable_browser",
        action="store_true",
        help="Disable opening information page on startup",
    )
    parser.add_argument(
        "--2d_vis",
        action="store_true",
        help="Enable 2D visualization when starting pylabrobot instance",
    )
    parser.add_argument(
        "--visual",
        choices=["rviz", "web", "disable"],
        default="disable",
        help="Choose visualization tool: rviz, web, or disable",
    )
    parser.add_argument(
        "--ak",
        type=str,
        default="",
        help="Access key for laboratory requests",
    )
    parser.add_argument(
        "--sk",
        type=str,
        default="",
        help="Secret key for laboratory requests",
    )
    parser.add_argument(
        "--addr",
        type=str,
        default="https://uni-lab.bohrium.com/api/v1",
        help="Laboratory backend address",
    )
    parser.add_argument(
        "--skip_env_check",
        action="store_true",
        help="Skip environment dependency check on startup",
    )
    parser.add_argument(
        "--complete_registry",
        action="store_true",
        default=False,
        help="Complete registry information",
    )
    return parser


def main():
    """主函数"""
    # 解析命令行参数
    args = parse_args()
    convert_argv_dashes_to_underscores(args)
    args_dict = vars(args.parse_args())

    # 环境检查 - 检查并自动安装必需的包 (可选)
    if not args_dict.get("skip_env_check", False):
        from unilabos.utils.environment_check import check_environment

        print_status("正在进行环境依赖检查...", "info")
        if not check_environment(auto_install=True):
            print_status("环境检查失败，程序退出", "error")
            os._exit(1)
    else:
        print_status("跳过环境依赖检查", "warning")

    # 加载配置文件，优先加载config，然后从env读取
    config_path = args_dict.get("config")
    if os.getcwd().endswith("unilabos_data"):
        working_dir = os.path.abspath(os.getcwd())
    else:
        working_dir = os.path.abspath(os.path.join(os.getcwd(), "unilabos_data"))
    if args_dict.get("working_dir"):
        working_dir = args_dict.get("working_dir", "")
        if config_path and not os.path.exists(config_path):
            config_path = os.path.join(working_dir, "local_config.py")
            if not os.path.exists(config_path):
                print_status(
                    f"当前工作目录 {working_dir} 未找到local_config.py，请通过 --config 传入 local_config.py 文件路径",
                    "error",
                )
                os._exit(1)
    elif config_path and os.path.exists(config_path):
        working_dir = os.path.dirname(config_path)
    elif os.path.exists(working_dir) and os.path.exists(os.path.join(working_dir, "local_config.py")):
        config_path = os.path.join(working_dir, "local_config.py")
    elif not config_path and (
        not os.path.exists(working_dir) or not os.path.exists(os.path.join(working_dir, "local_config.py"))
    ):
        print_status(f"未指定config路径，可通过 --config 传入 local_config.py 文件路径", "info")
        print_status(f"您是否为第一次使用？并将当前路径 {working_dir} 作为工作目录？ (Y/n)", "info")
        if input() != "n":
            os.makedirs(working_dir, exist_ok=True)
            config_path = os.path.join(working_dir, "local_config.py")
            shutil.copy(
                os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "example_config.py"), config_path
            )
            print_status(f"已创建 local_config.py 路径： {config_path}", "info")
        else:
            os._exit(1)
    # 加载配置文件
    print_status(f"当前工作目录为 {working_dir}", "info")
    load_config_from_file(config_path)
    if args_dict["addr"] == "test":
        print_status("使用测试环境地址", "info")
        HTTPConfig.remote_addr = "https://uni-lab.test.bohrium.com/api/v1"
    elif args_dict["addr"] == "uat":
        print_status("使用uat环境地址", "info")
        HTTPConfig.remote_addr = "https://uni-lab.uat.bohrium.com/api/v1"
    elif args_dict["addr"] == "local":
        print_status("使用本地环境地址", "info")
        HTTPConfig.remote_addr = "http://127.0.0.1:48197/api/v1"
    else:
        HTTPConfig.remote_addr = args_dict.get("addr", "")

    if args_dict["use_remote_resource"]:
        print_status("使用远程资源启动", "info")
        from unilabos.app.web import http_client

        res = http_client.resource_get("host_node", False)
        if str(res.get("code", 0)) == "0" and len(res.get("data", [])) > 0:
            print_status("远程资源已存在，使用云端物料！", "info")
            args_dict["graph"] = None
        else:
            print_status("远程资源不存在，本地将进行首次上报！", "info")

    # 设置BasicConfig参数
    if args_dict.get("ak", ""):
        BasicConfig.ak = args_dict.get("ak", "")
        print_status("传入了ak参数，优先采用传入参数！", "info")
    if args_dict.get("sk", ""):
        BasicConfig.sk = args_dict.get("sk", "")
        print_status("传入了sk参数，优先采用传入参数！", "info")
    BasicConfig.working_dir = working_dir
    BasicConfig.is_host_mode = not args_dict.get("is_slave", False)
    BasicConfig.slave_no_host = args_dict.get("slave_no_host", False)
    BasicConfig.upload_registry = args_dict.get("upload_registry", False)
    BasicConfig.communication_protocol = "websocket"
    machine_name = os.popen("hostname").read().strip()
    machine_name = "".join([c if c.isalnum() or c == "_" else "_" for c in machine_name])
    BasicConfig.machine_name = machine_name
    BasicConfig.vis_2d_enable = args_dict["2d_vis"]

    from unilabos.resources.graphio import (
        read_node_link_json,
        read_graphml,
        dict_from_graph,
        dict_to_nested_dict,
        initialize_resources,
    )
    from unilabos.app.communication import get_communication_client
    from unilabos.registry.registry import build_registry
    from unilabos.app.backend import start_backend
    from unilabos.app.web import http_client
    from unilabos.app.web import start_server
    from unilabos.app.register import register_devices_and_resources

    # 显示启动横幅
    print_unilab_banner(args_dict)

    # 注册表
    lab_registry = build_registry(
        args_dict["registry_path"], args_dict.get("complete_registry", False), args_dict["upload_registry"]
    )

    if not BasicConfig.ak or not BasicConfig.sk:
        print_status("后续运行必须拥有一个实验室，请前往 https://uni-lab.bohrium.com 注册实验室！", "warning")
        os._exit(1)
    if args_dict["graph"] is None:
        request_startup_json = http_client.request_startup_json()
        if not request_startup_json:
            print_status(
                "未指定设备加载文件路径，尝试从HTTP获取失败，请检查网络或者使用-g参数指定设备加载文件路径", "error"
            )
            os._exit(1)
        else:
            print_status("联网获取设备加载文件成功", "info")
        graph, data = read_node_link_json(request_startup_json)
    else:
        file_path = args_dict["graph"]
        if file_path.endswith(".json"):
            graph, data = read_node_link_json(file_path)
        else:
            graph, data = read_graphml(file_path)
    import unilabos.resources.graphio as graph_res

    graph_res.physical_setup_graph = graph
    resource_edge_info = modify_to_backend_format(data["links"])
    materials = lab_registry.obtain_registry_resource_info()
    materials.extend(lab_registry.obtain_registry_device_info())
    materials = {k["id"]: k for k in materials}
    nodes = {k["id"]: k for k in data["nodes"]}
    edge_info = len(resource_edge_info)
    for ind, i in enumerate(resource_edge_info[::-1]):
        source_node = nodes[i["source"]]
        target_node = nodes[i["target"]]
        source_handle = i["sourceHandle"]
        target_handle = i["targetHandle"]
        source_handler_keys = [
            h["handler_key"] for h in materials[source_node["class"]]["handles"] if h["io_type"] == "source"
        ]
        target_handler_keys = [
            h["handler_key"] for h in materials[target_node["class"]]["handles"] if h["io_type"] == "target"
        ]
        if source_handle not in source_handler_keys:
            print_status(
                f"节点 {source_node['id']} 的source端点 {source_handle} 不存在，请检查，支持的端点 {source_handler_keys}",
                "error",
            )
            resource_edge_info.pop(edge_info - ind - 1)
            continue
        if target_handle not in target_handler_keys:
            print_status(
                f"节点 {target_node['id']} 的target端点 {target_handle} 不存在，请检查，支持的端点 {target_handler_keys}",
                "error",
            )
            resource_edge_info.pop(edge_info - ind - 1)
            continue

    devices_and_resources = dict_from_graph(graph_res.physical_setup_graph)
    # args_dict["resources_config"] = initialize_resources(list(deepcopy(devices_and_resources).values()))
    args_dict["resources_config"] = list(devices_and_resources.values())
    args_dict["devices_config"] = dict_to_nested_dict(deepcopy(devices_and_resources), devices_only=False)
    args_dict["graph"] = graph_res.physical_setup_graph

    print_status(f"{len(args_dict['resources_config'])} Resources loaded:", "info")
    for i in args_dict["resources_config"]:
        print_status(f"DeviceId: {i['id']}, Class: {i['class']}", "info")

    if BasicConfig.upload_registry:
        # 设备注册到服务端 - 需要 ak 和 sk
        if args_dict.get("ak") and args_dict.get("sk"):
            print_status("开始注册设备到服务端...", "info")
            try:
                register_devices_and_resources(lab_registry)
                print_status("设备注册完成", "info")
            except Exception as e:
                print_status(f"设备注册失败: {e}", "error")
        else:
            print_status("未提供 ak 和 sk，跳过设备注册", "info")
    else:
        print_status(
            "本次启动注册表不报送云端，如果您需要联网调试，请在启动命令增加--upload_registry", "warning"
        )

    if args_dict["controllers"] is not None:
        args_dict["controllers_config"] = yaml.safe_load(open(args_dict["controllers"], encoding="utf-8"))
    else:
        args_dict["controllers_config"] = None

    args_dict["bridges"] = []

    # 获取通信客户端（仅支持WebSocket）
    comm_client = get_communication_client()

    if "websocket" in args_dict["app_bridges"]:
        args_dict["bridges"].append(comm_client)
    if "fastapi" in args_dict["app_bridges"]:
        args_dict["bridges"].append(http_client)
    if "websocket" in args_dict["app_bridges"]:

        def _exit(signum, frame):
            comm_client.stop()
            sys.exit(0)

        signal.signal(signal.SIGINT, _exit)
        signal.signal(signal.SIGTERM, _exit)
        comm_client.start()
    args_dict["resources_mesh_config"] = {}
    args_dict["resources_edge_config"] = resource_edge_info
    # web visiualize 2D
    if args_dict["visual"] != "disable":
        enable_rviz = args_dict["visual"] == "rviz"
        if devices_and_resources is not None:
            from unilabos.device_mesh.resource_visalization import (
                ResourceVisualization,
            )  # 此处开启后，logger会变更为INFO，有需要请调整

            resource_visualization = ResourceVisualization(
                devices_and_resources, args_dict["resources_config"], enable_rviz=enable_rviz
            )
            args_dict["resources_mesh_config"] = resource_visualization.resource_model
            start_backend(**args_dict)
            server_thread = threading.Thread(
                target=start_server,
                kwargs=dict(
                    open_browser=not args_dict["disable_browser"],
                    port=args_dict["port"],
                ),
            )
            server_thread.start()
            asyncio.set_event_loop(asyncio.new_event_loop())
            resource_visualization.start()
            while True:
                time.sleep(1)
        else:
            start_backend(**args_dict)
            start_server(
                open_browser=not args_dict["disable_browser"],
                port=args_dict["port"],
            )
    else:
        start_backend(**args_dict)
        start_server(
            open_browser=not args_dict["disable_browser"],
            port=args_dict["port"],
        )


if __name__ == "__main__":
    main()
