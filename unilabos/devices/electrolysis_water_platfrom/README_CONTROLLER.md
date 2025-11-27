# 电解水平台 + 控制器使用说明

## 🎯 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                    电解水平台                                  │
│                                                               │
│  1. 串口接收数据 → parse_rx_payload() 解析                      │
│  2. 存储到 _latest_data                                        │
│  3. get_sensor_data() 返回列表                                │
│                                                               │
│          ↓ 传感器数据列表                                      │
│          │ [current, voltage, temp, ph, gas, liquid]         │
│          ↓                                                    │
│                                                               │
│  ┌─────────────────────────────────────────────┐             │
│  │           控制器 (Controller)                 │             │
│  │                                              │             │
│  │  1. 接收传感器数据                             │             │
│  │  2. 传入深度学习模型                           │             │
│  │  3. 模型推理计算                              │             │
│  │  4. 返回控制向量                              │             │
│  └─────────────────────────────────────────────┘             │
│                                                               │
│          ↑ 控制向量列表                                        │
│          │ [mode, current_ma, voltage_mv, temp_c, pump%]     │
│          ↓                                                    │
│                                                               │
│  4. _execute_control_vector() 解析控制向量                     │
│  5. send_command() 发送控制指令                                │
│  6. build_tx_frame() 构建数据帧                                │
│  7. 串口发送                                                   │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## 📦 核心组件

### 1. 电解水平台 (`ElectrolysisWaterPlatform`)

**职责**：
- 解析传感器数据（`parse_rx_payload`）
- 提供数据接口（`get_sensor_data`）
- 执行控制指令（`send_command`）
- 管理控制循环

**关键方法**：
```python
# 获取传感器数据（列表格式）
def get_sensor_data() -> list:
    return [timestamp, current, voltage, temp, tds, gas, liquid, ph]

# 发送控制指令
def send_command(mode, current_ma, voltage_mv, temp_c, ki, pump_percent) -> bool

# 设置控制器
def set_controller(controller)

# 启动/停止控制循环
def start_control_loop()
def stop_control_loop()
```

### 2. 控制器 (`ElectrolysisController`)

**职责**：
- 接收传感器数据
- 使用深度学习模型计算
- 返回控制向量

**关键方法**：
```python
# 计算控制向量
def compute_control(sensor_data: List[float]) -> List[float]:
    # 输入: [current, voltage, temp, ph, gas, liquid]
    # 输出: [mode, current_ma, voltage_mv, temp_c, pump_percent]
    
# 设置归一化参数
def set_normalization_params(input_mean, input_std, output_mean, output_std)
```

## 🚀 使用方法

### 方式 1：使用控制器（推荐）

```python
from electrolysis_water_platfrom import ElectrolysisWaterPlatform
from electrolysis_controller import ElectrolysisController
import time

# 1. 创建平台（不自动控制）
platform = ElectrolysisWaterPlatform(
    port="COM5",
    baudrate=115200,
    sampling_interval=1.0,
    auto_start_control=False  # 由控制器接管
)

# 等待连接
time.sleep(3)

# 2. 创建控制器
controller = ElectrolysisController(
    model_path="model.pth",  # 可选：模型路径
    model_type="pytorch",    # pytorch / tensorflow / dummy
    input_dim=6,
    output_dim=5
)

# 3. 可选：设置归一化参数
controller.set_normalization_params(
    input_mean=[1000, 2500, 25, 7, 50, 30],
    input_std=[500, 1000, 5, 2, 25, 15],
    output_mean=[0, 1000, 2500, 25, 50],
    output_std=[1, 500, 1000, 5, 25]
)

# 4. 将控制器设置到平台
platform.set_controller(controller)

# 5. 启动控制循环
platform.start_control_loop()

# 6. 监控运行
try:
    while True:
        data = platform.get_sensor_data()
        print(f"I={data[1]:.1f}mA, V={data[2]:.1f}mV, T={data[3]:.2f}°C")
        time.sleep(1)
except KeyboardInterrupt:
    platform.stop_control_loop()
    platform.stop()
```

### 方式 2：手动控制（不使用控制器）

```python
from electrolysis_water_platfrom import ElectrolysisWaterPlatform
import time

# 创建平台（自动发送初始控制指令）
platform = ElectrolysisWaterPlatform(
    port="COM5",
    baudrate=115200,
    default_voltage_mv=2000,
    default_current_ma=1000,
    auto_start_control=True  # 自动控制
)

time.sleep(3)

# 手动发送控制指令
platform.send_command(
    mode=0,           # 恒压
    current_ma=1000,
    voltage_mv=2500,
    pump_percent=60.0
)

# 监控数据
for i in range(20):
    data = platform.get_sensor_data()
    print(f"数据: {data}")
    time.sleep(1)

platform.stop()
```

## 🎓 数据格式

### 输入数据（传感器）

```python
# get_sensor_data() 返回:
[
    timestamp,      # str: "2024-01-01 12:00:00"
    current_mA,     # float: 电流 (mA)
    voltage_mV,     # float: 电压 (mV)
    temperature_C,  # float: 温度 (°C)
    tds_ppm,       # float: TDS (ppm) - 不用于控制
    gas_flow_sccm, # float: 气体流量 (sccm)
    liquid_flow_mL,# float: 液体流量 (mL)
    ph             # float: pH值
]

# 传给控制器的数据（去掉timestamp和tds）:
[current_mA, voltage_mV, temperature_C, ph, gas_flow_sccm, liquid_flow_mL]
```

### 输出数据（控制向量）

```python
# controller.compute_control() 返回:
[
    mode,           # int: 0=恒压, 1=恒流
    current_ma,     # float: 目标电流 (mA)
    voltage_mv,     # float: 目标电压 (mV)
    temperature_c,  # float: 目标温度 (°C)
    pump_percent    # float: 泵速 (%)
]
```

## 🧠 深度学习模型

### 创建模型

```python
# PyTorch 示例
import torch
import torch.nn as nn

class ControlNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(6, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Linear(32, 5)
        )
    
    def forward(self, x):
        return self.network(x)

# 训练后保存
model = ControlNet()
# ... 训练代码 ...
torch.save(model, "electrolysis_model.pth")
```

### 使用模型

```python
controller = ElectrolysisController(
    model_path="electrolysis_model.pth",
    model_type="pytorch",
    device="cpu"  # 或 "cuda"
)
```

## 📝 配置示例

### 使用配置字典

```python
config = {
    "port": "COM5",
    "baudrate": 115200,
    "default_voltage_mv": 2500,
    "default_current_ma": 1200,
    "sampling_interval": 0.5,
    "auto_start_control": False
}

platform = ElectrolysisWaterPlatform(config=config)
```

## ⚠️ 注意事项

1. **安全限幅**：控制向量会自动限幅
   - 电流: 0-2000 mA
   - 电压: 0-5000 mV
   - 温度: 0-50 °C
   - 泵速: 0-100 %

2. **采样频率**：建议 `sampling_interval >= 0.5` 秒

3. **模型要求**：
   - 输入维度: 6 (current, voltage, temp, ph, gas, liquid)
   - 输出维度: 5 (mode, current, voltage, temp, pump)

4. **归一化**：如果模型训练时使用了归一化，必须设置归一化参数

## 🔧 故障排查

### 问题1：控制循环无法启动

```python
# 确保已设置控制器
platform.set_controller(controller)
platform.start_control_loop()
```

### 问题2：数据全为0

```python
# 检查串口连接
print(platform.connection_status)  # 应该显示 "Connected"

# 等待数据接收
time.sleep(2)
data = platform.get_sensor_data()
print(data)
```

### 问题3：模型加载失败

```python
# 使用简单规则（不加载模型）
controller = ElectrolysisController(
    model_path=None,  # 不加载模型
    model_type="dummy"
)
```

## 📊 完整示例

见 `electrolysis_water_platfrom.py` 和 `electrolysis_controller.py` 文件末尾的示例代码。

直接运行：
```bash
python electrolysis_water_platfrom.py
```

或

```bash
python electrolysis_controller.py
```

