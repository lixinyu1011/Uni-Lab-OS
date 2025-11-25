# 电解水平台自动控制系统

## 概述

本文档介绍如何使用自动控制系统来管理电解水平台的运行。系统支持恒压（CV）和恒流（CC）两种控制模式，可以实现：

- 基于 PID 的闭环控制
- 实时监控和数据记录
- 安全限制和报警
- 多变量协同控制

## 文件说明

### 配置文件
- `electrolysis_control_config.yaml` - 控制器配置文件
- `deis_control_config.yaml` - 参考配置示例

### 代码文件
- `electrolysis_controller.py` - 控制器实现
- `electrolysis_water_platfrom.py` - 电解水平台驱动
- `electrolysis_deck.py` - Deck 资源定义

## 控制模式

### 1. 恒压模式 (Constant Voltage, CV)

保持输出电压恒定，电流根据负载自动调整。

**配置示例**:
```yaml
voltage_controller:
  parameters:
    mode: 0  # 恒压模式
    target_voltage: 3500  # 目标电压 3.5V
    max_current: 2000     # 最大电流 2A
```

**适用场景**:
- 电解液电阻较稳定时
- 需要控制电压避免过电势
- 初始阶段建立稳定状态

### 2. 恒流模式 (Constant Current, CC)

保持输出电流恒定，电压根据负载自动调整。

**配置示例**:
```yaml
current_controller:
  parameters:
    mode: 1  # 恒流模式
    target_current: 1500  # 目标电流 1.5A
    max_voltage: 5000     # 最大电压 5V
```

**适用场景**:
- 要求固定产气速率
- 电极表面积恒定
- 长时间稳定运行

## 控制帧格式

发送到下位机的控制帧格式：

```
HEAD(1字节) + DATA(9字节) + TAIL(1字节) = 11字节

DATA 组成:
- mode (1字节): 0=恒压, 1=恒流
- current_hi (1字节): 电流高字节
- current_lo (1字节): 电流低字节
- voltage_hi (1字节): 电压高字节
- voltage_lo (1字节): 电压低字节
- temp_hi (1字节): 温度高字节
- temp_lo (1字节): 温度低字节
- ki (1字节): Ki参数 (0-20.0, 编码为 0-200)
- pump (1字节): 泵速 (0-100%, 编码为 0-200)
```

**示例**:
- 恒压 3.5V, 最大1A: `mode=0, voltage=3500, current=1000`
- 恒流 1.5A, 最大5V: `mode=1, current=1500, voltage=5000`

## 使用方法

### 方法 1: 使用配置文件（推荐）

1. 编辑配置文件 `electrolysis_control_config.yaml`
2. 选择要激活的控制器:
   ```yaml
   active_controllers:
     - voltage_controller  # 激活恒压控制
   ```

3. 启动控制系统:
   ```bash
   python -m unilabos.app.main -g WE.json --controller electrolysis_control_config.yaml
   ```

### 方法 2: Python API

```python
from unilabos.devices.electrolysis_water_platfrom.electrolysis_water_platfrom import ElectrolysisWaterPlatform
from unilabos.devices.electrolysis_water_platfrom.electrolysis_controller import ElectrolysisController

# 创建平台实例
platform = ElectrolysisWaterPlatform(port="COM5")

# 配置控制器
config = {
    'mode': 0,              # 0=恒压, 1=恒流
    'target_voltage': 3500, # mV
    'target_current': 1000, # mA
    'target_temperature': 25.0,
    'kp': 0.5,             # PID 参数
    'ki': 0.1,
    'kd': 0.05,
    'control_rate': 1.0    # Hz
}

# 创建并启动控制器
controller = ElectrolysisController(platform, config)
controller.start_control()

# 动态调整目标值
controller.set_target(voltage=4000, current=1200)

# 切换控制模式
controller.switch_mode(1)  # 切换到恒流模式

# 获取统计信息
stats = controller.get_statistics()
print(stats)

# 停止控制
controller.stop_control()
```

### 方法 3: 交互式命令

运行控制器测试程序:
```bash
cd c:\ML\GitHub\Uni-Lab-OS
python -m unilabos.devices.electrolysis_water_platfrom.electrolysis_controller
```

可用命令:
- `set <voltage> <current>` - 设置目标值
  - 例如: `set 3500 1000`
- `mode <0|1>` - 切换模式
  - `mode 0` - 恒压模式
  - `mode 1` - 恒流模式
- `stats` - 显示统计信息
- `stop` - 停止控制

## PID 参数调整

### 参数说明

- **Kp (比例系数)**: 响应速度，越大响应越快但可能振荡
- **Ki (积分系数)**: 消除稳态误差，越大收敛越快但可能超调
- **Kd (微分系数)**: 抑制振荡，改善稳定性

### 调整建议

#### 恒压模式
```yaml
kp: 0.5   # 中等响应速度
ki: 0.1   # 较慢积分避免超调
kd: 0.05  # 小量微分稳定系统
```

#### 恒流模式
```yaml
kp: 1.0   # 较快响应
ki: 0.2   # 中等积分
kd: 0.1   # 中等微分
```

### 调整步骤

1. **只设置 Kp**:
   - 从小值开始 (0.1)
   - 逐渐增大直到出现小幅振荡
   - 减小到振荡消失

2. **添加 Ki**:
   - 从很小值开始 (0.01)
   - 逐渐增大以消除稳态误差
   - 过大会导致超调

3. **添加 Kd**:
   - 从 0 开始
   - 如有振荡，逐渐增大 Kd
   - 过大会放大噪声

## 安全限制

系统内置安全保护:

```yaml
safety_limits:
  max_voltage: 5000    # 最大电压 5V
  max_current: 2000    # 最大电流 2A
  max_temperature: 80  # 最大温度 80°C
```

### 报警条件

- **过电流**: 超过 2000mA 自动降压
- **过电压**: 超过 5000mV 自动降流
- **过温**: 超过 60°C 紧急停止
- **异常 pH**: 超出 2.0-12.0 范围暂停并报警

## 数据记录

### 自动记录

启用数据记录:
```yaml
data_logging:
  enabled: true
  log_file: "electrolysis_control_log.csv"
  log_rate: 1.0  # Hz
```

记录字段:
- 时间戳
- 电流、电压、温度
- pH值、气体流量、液体流量
- 控制模式和设定点

### 手动记录

传感器数据自动保存到 CSV 文件:
```
stm32_data.csv
```

格式:
```csv
Timestamp,Current_mA,Voltage_mV,Temperature_C,TDS_ppm,GasFlow_sccm,LiquidFlow_mL,pH
2025-11-25 20:45:35,1000,3500,25.5,450,100,50,7.2
```

## 故障排查

### 问题 1: 控制器无响应

**可能原因**:
- 串口未连接
- 控制频率过低
- PID 参数不合适

**解决方法**:
```python
# 检查连接
print(platform.connection_status)  # 应该显示 "Connected"

# 增加控制频率
config['control_rate'] = 2.0  # 提高到 2Hz

# 重置 PID
controller.pid.reset()
```

### 问题 2: 振荡严重

**可能原因**:
- Kp 过大
- Kd 过小

**解决方法**:
```python
# 降低比例系数
controller.pid.set_tunings(kp=0.3, ki=0.1, kd=0.1)
```

### 问题 3: 响应过慢

**可能原因**:
- Kp 过小
- Ki 过小

**解决方法**:
```python
# 增加比例和积分系数
controller.pid.set_tunings(kp=0.8, ki=0.15, kd=0.05)
```

## 高级功能

### 多变量控制

同时控制多个参数:
```yaml
multi_variable_controller:
  parameters:
    target_voltage: 3500
    target_current: 1200
    target_temperature: 28.0
    target_ph: 7.0
    optimization_enabled: true
```

### 自适应控制

根据系统响应自动调整 PID 参数（待实现）。

### 远程监控

通过 ROS2 话题实时监控:
```bash
# 监控电流
ros2 topic echo /devices/electrolysis_water_platfrom/current

# 监控电压
ros2 topic echo /devices/electrolysis_water_platfrom/voltage
```

## 参考资料

- [PID控制原理](https://en.wikipedia.org/wiki/PID_controller)
- [电解水反应机理](https://en.wikipedia.org/wiki/Electrolysis_of_water)
- UniLab-OS 文档: `docs/`

## 联系支持

如有问题请联系开发团队或提交 Issue。

