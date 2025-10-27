# 电解水平台 (Electrolysis Water Platform)

## 功能说明

这是一个基于 `WorkstationBase` 的电解水实验平台，支持：
- 串口通信控制下位机（STM32）
- 实时数据采集与本地CSV存储
- **实时数据上传到云端显示**
- 支持恒压/恒流两种工作模式

## 实时数据云端上报

### 上报的数据类型

系统会自动将以下传感器数据实时上传到云端（默认1秒更新一次）：

| 数据类型 | 说明 | 单位 | 更新频率 |
|---------|------|------|---------|
| `current` | 电流值 | mA | 1秒 |
| `voltage` | 电压值 | mV | 1秒 |
| `temperature` | 温度值 | ℃ | 1秒 |
| `tds` | TDS值 | ppm | 1秒 |
| `gas_flow` | 气体流量 | sccm | 1秒 |
| `liquid_flow` | 液体流量 | mL | 1秒 |
| `ph` | pH值 | - | 1秒 |
| `connection_status` | 连接状态 | Boolean | 5秒 |
| `timestamp` | 最后更新时间 | String | 1秒 |

### 工作原理

1. **数据采集**：接收线程从串口读取下位机数据
2. **本地存储**：数据保存到CSV文件（默认：`stm32_data.csv`）
3. **实时缓存**：最新数据存储在内存中（线程安全）
4. **自动发布**：ROS2系统自动调用`get_xxx()`方法获取数据
5. **云端上报**：通过 Bridge 将数据发送到云端服务器
6. **Web显示**：云端Web界面实时展示数据曲线

### 配置说明

在设备配置文件中配置平台：

```json
{
  "nodes": [{
    "id": "electrolysis_platform_001",
    "class": "electrolysiswaterplatform_device",
    "config": {
      "port": "COM5",           // 串口号
      "baudrate": 115200,       // 波特率
      "timeout": 0.2,           // 超时时间
      "csv_path": null          // CSV路径（null=自动使用代码目录）
    }
  }]
}
```

### 数据格式

#### 接收帧格式（下位机→上位机）
- 固定15字节：`HEAD(1) + DATA(13) + TAIL(1)`
- HEAD = 0x3E, TAIL = 0x3E
- DATA包含：电流、电压、温度、TDS、气体流量、液体流量、pH

#### 发送帧格式（上位机→下位机）
- 固定11字节：`HEAD(1) + DATA(9) + TAIL(1)`
- HEAD = 0x3E, TAIL = 0xE3
- DATA包含：模式、电流设定、电压设定、温度设定、Ki、泵速

## 使用示例

### 1. 在系统中使用（自动云端上报）

```python
from pylabrobot.resources import Deck
from unilabos.devices.electrolysis_water_platfrom.electrolysis_water_platfrom import ElectrolysisWaterPlatform

# 创建平台实例
deck = Deck()
platform = ElectrolysisWaterPlatform(
    deck,
    port="COM5",
    baudrate=115200
)

# 启动平台（开启接收和发送线程）
platform.start()

# 在另一个线程/进程中，可以获取实时数据
data = platform.get_latest_data()
print(f"当前电流: {data['Current_mA']} mA")
print(f"当前电压: {data['Voltage_mV']} mV")
print(f"当前温度: {data['Temperature_C']} ℃")

# 发送控制命令
platform.send_command(
    mode=1,           # 恒流模式
    current_ma=2000,  # 2000mA
    voltage_mv=500,   # 500mV
    temp_c=25.0,      # 25℃
    ki=0.0,
    pump_percent=100.0
)

# 停止平台
platform.stop()
```

### 2. 独立运行（仅本地采集）

```python
python electrolysis_water_platfrom.py
```

然后按提示输入控制命令，格式：
```
mode,current_mA,voltage_mV,set_temp_C,Ki,pump_percent
```

示例：
- 恒压模式：`0,500,1000,25,0,100`
- 恒流模式：`1,2000,500,25,0,100`

输入 `stop` 结束程序。

### 3. 通过云端控制

在云端Web界面中：
1. 找到设备 `electrolysis_platform_001`
2. 实时查看传感器数据曲线
3. 通过 Action 发送控制命令：
   - 选择 `build_tx_frame` 动作
   - 填写参数并执行

## 数据文件

### CSV文件格式

```csv
Timestamp,Current_mA,Voltage_mV,Temperature_C,TDS_ppm,GasFlow_sccm,LiquidFlow_mL,pH
2025-10-27 10:30:01,2000,500,25.45,120,150,80,7.2
2025-10-27 10:30:02,2001,501,25.48,121,151,81,7.3
...
```

### 文件位置

- 默认位置：`unilabos/devices/electrolysis_water_platfrom/stm32_data.csv`
- 自定义位置：通过 `csv_path` 参数指定

## API参考

### 主要方法

#### `open_serial(port, baudrate, timeout)`
打开串口连接

#### `close_serial()`
关闭串口连接

#### `start()`
启动电解水平台（开启接收和发送线程）

#### `stop()`
停止电解水平台

#### `send_command(mode, current_ma, voltage_mv, temp_c, ki, pump_percent)`
发送控制命令到下位机

### 实时数据获取方法

#### `get_latest_data() -> Dict`
获取所有最新传感器数据

#### `get_current() -> float`
获取当前电流值（mA）

#### `get_voltage() -> float`
获取当前电压值（mV）

#### `get_temperature() -> float`
获取当前温度值（℃）

#### `get_tds() -> float`
获取当前TDS值（ppm）

#### `get_gas_flow() -> float`
获取当前气体流量（sccm）

#### `get_liquid_flow() -> float`
获取当前液体流量（mL）

#### `get_ph() -> float`
获取当前pH值

#### `get_connection_status() -> bool`
获取串口连接状态

#### `get_timestamp() -> str`
获取最后更新时间

## 注意事项

1. **线程安全**：所有数据访问都使用锁保护，支持多线程并发访问
2. **数据同步**：CSV文件每次接收数据后立即flush，确保数据不丢失
3. **错误处理**：串口异常会自动更新连接状态为False
4. **云端延迟**：数据上报有1秒的采样周期，实时性取决于网络延迟
5. **端口配置**：确保串口号正确，Windows使用`COM5`，Linux使用`/dev/ttyUSB0`

## 故障排查

### 问题：串口无法打开
- 检查串口号是否正确
- 检查串口是否被其他程序占用
- 检查串口权限（Linux需要`sudo`或添加到`dialout`组）

### 问题：数据不更新
- 检查下位机是否正常发送数据
- 检查波特率是否匹配（115200）
- 查看CSV文件是否有新数据写入

### 问题：云端无数据
- 检查设备是否在云端注册成功
- 检查Bridge连接是否正常
- 查看ROS2节点是否正常运行：`ros2 topic list`
- 查看数据发布：`ros2 topic echo /devices/electrolysis_platform_001/current`

## 版本历史

### v1.0.0 (2025-10-27)
- ✅ 基础串口通信功能
- ✅ 本地CSV数据存储
- ✅ 实时数据云端上报
- ✅ 恒压/恒流模式控制
- ✅ 线程安全的数据访问
- ✅ 完整的API接口

