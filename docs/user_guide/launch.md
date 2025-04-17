# Uni-Lab 启动

安装完毕后，可以通过 `unilab` 命令行启动：

```bash
Start Uni-Lab Edge server.

options:
  -h, --help            show this help message and exit
  -g GRAPH, --graph GRAPH
                        Physical setup graph.
  -d DEVICES, --devices DEVICES
                        Devices config file.
  -r RESOURCES, --resources RESOURCES
                        Resources config file.
  -c CONTROLLERS, --controllers CONTROLLERS
                        Controllers config file.
  --registry_path REGISTRY_PATH
                        Path to the registry
  --backend {ros,simple,automancer}
                        Choose the backend to run with: 'ros', 'simple', or 'automancer'.
  --app_bridges APP_BRIDGES [APP_BRIDGES ...]
                        Bridges to connect to. Now support 'mqtt' and 'fastapi'.
  --without_host        Run the backend as slave (without host).
  --config CONFIG       Configuration file path for system settings
```

## 使用配置文件

Uni-Lab支持使用Python格式的配置文件进行系统设置。通过 `--config` 参数指定配置文件路径：

```bash
# 使用配置文件启动
unilab --config path/to/your/config.py
```

配置文件包含MQTT、HTTP、ROS等系统设置。有关配置文件的详细信息，请参阅[配置指南](configuration.md)。

## 初始化信息来源

启动 Uni-Lab 时，可以选用两种方式之一配置实验室设备、耗材、通信、控制逻辑：

### 1. 组态&拓扑图

使用 `-g` 时，组态&拓扑图应包含实验室所有信息，详见{ref}`graph`。目前支持 graphml 和 node-link json 两种格式。格式可参照 `tests/experiments` 下的启动文件。

### 2. 分别指定设备、耗材、控制逻辑

分别使用 `-d, -r, -c` 依次传入设备组态配置、耗材列表、控制逻辑。

可参照 `devices.json` 和 `resources.json`。

不管使用哪一种初始化方式，设备/物料字典均需包含 `class` 属性，用于查找注册表信息。默认查找范围都是 Uni-Lab 内部注册表 `unilabos/registry/{devices,device_comms,resources}`。要添加额外的注册表路径，可以使用 `--registry` 加入 `<your-registry-path>/{devices,device_comms,resources}`。

## 通信中间件 `--backend`

目前 Uni-Lab 仅支持 ros2 作为通信中间件。

## 端云桥接 `--app_bridges`

目前 Uni-Lab 提供 FastAPI (http), MQTT 两种端云通信方式。其中默认 MQTT 负责端对云状态同步和云对端任务下发，FastAPI 负责端对云物料更新。

## 分布式组网

启动 Uni-Lab 时，加入 `--without_host` 将作为从站，不加将作为主站，主站 (host) 持有物料修改权以及对云端的通信。局域网内分别启动的 Uni-Lab 主站/从站将自动组网，互相能访问所有设备状态、传感器信息并发送指令。

## 完整启动示例

以下是一些常用的启动命令示例：

```bash
# 使用配置文件和组态图启动
unilab -g path/to/graph.json

# 使用配置文件和分离的设备/资源文件启动
unilab -d devices.json -r resources.json
```
