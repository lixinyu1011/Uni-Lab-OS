# 电解水平台深度学习控制器 - 快速入门

## 概述

本控制器使用深度学习模型替代传统 PID 控制，实现更智能的电解过程控制。

## 核心特性

✅ **模型灵活性**: 支持 PyTorch 和 TensorFlow 模型  
✅ **实时推理**: 毫秒级响应时间  
✅ **自动归一化**: 内置数据预处理  
✅ **安全保护**: 输出限幅和异常处理  

## 快速开始

### 1. 安装依赖

选择一个深度学习框架：

```bash
# 使用 PyTorch（推荐）
pip install torch numpy

# 或使用 TensorFlow
pip install tensorflow numpy
```

### 2. 准备模型

#### 选项 A: 使用示例模型

运行示例代码创建简单模型：

```bash
cd c:\ML\GitHub\Uni-Lab-OS
python -m unilabos.devices.electrolysis_water_platfrom.electrolysis_controller
```

这将创建：
- `electrolysis_model.pth` (PyTorch)
- `electrolysis_model_tf/` (TensorFlow)

#### 选项 B: 使用自己的模型

**模型要求**：
- 输入维度: 6 (电流、电压、温度、pH、气流、液流)
- 输出维度: 2 (控制电压、控制电流)

**PyTorch 示例**：
```python
import torch
import torch.nn as nn

class MyControlModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(6, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Linear(32, 2)
        )
    
    def forward(self, x):
        return self.net(x)

# 保存模型
model = MyControlModel()
torch.save(model, "my_model.pth")
```

**TensorFlow 示例**：
```python
import tensorflow as tf

model = tf.keras.Sequential([
    tf.keras.layers.Dense(32, activation='relu', input_shape=(6,)),
    tf.keras.layers.Dense(32, activation='relu'),
    tf.keras.layers.Dense(2)
])

# 保存模型
model.save("my_model_tf")
```

### 3. 配置控制器

编辑 `electrolysis_control_config.yaml`:

```yaml
voltage_controller:
  parameters:
    mode: 0  # 0=恒压, 1=恒流
    target_voltage: 3500
    target_current: 1000
    
    model:
      path: "models/electrolysis_cv_model.pth"
      type: "pytorch"  # 或 "tensorflow"
      device: "cpu"    # 或 "cuda"
      
      input_features:
        - current
        - voltage
        - temperature
        - ph
        - gas_flow
        - liquid_flow
      
      output_features:
        - control_voltage
        - control_current
```

### 4. 运行控制器

```python
from unilabos.devices.electrolysis_water_platfrom.electrolysis_water_platfrom import ElectrolysisWaterPlatform
from unilabos.devices.electrolysis_water_platfrom.electrolysis_controller import ElectrolysisController

# 创建平台
platform = ElectrolysisWaterPlatform(port="COM5")

# 配置控制器
config = {
    'mode': 0,
    'target_voltage': 3500,
    'target_current': 1000,
    'model': {
        'path': 'electrolysis_model.pth',
        'type': 'pytorch',
        'device': 'cpu'
    }
}

# 创建并启动控制器
controller = ElectrolysisController(platform, config)
controller.start_control()

# 运行一段时间...
import time
time.sleep(60)

# 停止控制
controller.stop_control()
```

## 数据归一化

模型训练时使用的数据范围需要在配置中指定：

```yaml
normalization:
  input_mean: [1000, 3500, 25, 7, 50, 30]
  input_std: [500, 1000, 5, 2, 25, 15]
  output_mean: [3500, 1000]
  output_std: [1000, 500]
```

公式：`normalized = (value - mean) / std`

## 模型输入输出说明

### 输入特征 (6维)

| 索引 | 特征 | 单位 | 典型范围 |
|-----|------|------|---------|
| 0 | current | mA | 0-2000 |
| 1 | voltage | mV | 0-5000 |
| 2 | temperature | ℃ | 10-80 |
| 3 | ph | - | 0-14 |
| 4 | gas_flow | sccm | 0-200 |
| 5 | liquid_flow | mL | 0-100 |

### 输出特征 (2维)

| 索引 | 特征 | 单位 | 典型范围 |
|-----|------|------|---------|
| 0 | control_voltage | mV | 0-5000 |
| 1 | control_current | mA | 0-2000 |

## 模型训练建议

### 数据采集

1. **多工况采集**: 不同电压、电流、温度组合
2. **长时间运行**: 捕捉动态特性
3. **包含扰动**: 负载变化、温度波动

### 训练流程

```python
import torch
import torch.nn as nn
import numpy as np

# 1. 加载数据
X_train = np.loadtxt("training_data_input.csv", delimiter=",")
y_train = np.loadtxt("training_data_output.csv", delimiter=",")

# 2. 归一化
input_mean = X_train.mean(axis=0)
input_std = X_train.std(axis=0)
X_train = (X_train - input_mean) / input_std

output_mean = y_train.mean(axis=0)
output_std = y_train.std(axis=0)
y_train = (y_train - output_mean) / output_std

# 3. 创建数据集
train_dataset = torch.utils.data.TensorDataset(
    torch.FloatTensor(X_train),
    torch.FloatTensor(y_train)
)
train_loader = torch.utils.data.DataLoader(
    train_dataset, batch_size=32, shuffle=True
)

# 4. 训练模型
model = MyControlModel()
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
criterion = nn.MSELoss()

for epoch in range(100):
    for X_batch, y_batch in train_loader:
        optimizer.zero_grad()
        y_pred = model(X_batch)
        loss = criterion(y_pred, y_batch)
        loss.backward()
        optimizer.step()

# 5. 保存模型和归一化参数
torch.save(model, "my_trained_model.pth")
np.save("normalization_params.npy", {
    'input_mean': input_mean,
    'input_std': input_std,
    'output_mean': output_mean,
    'output_std': output_std
})
```

## 高级功能

### 使用 GPU 加速

```yaml
model:
  device: "cuda"  # 使用 GPU
```

### 自定义输入特征

如果只需要部分传感器数据：

```yaml
model:
  input_features:
    - current
    - voltage
    - temperature
  # 不使用 ph, gas_flow, liquid_flow
```

### 添加历史数据

模型可以使用历史状态（需要修改模型结构）：

```python
class SequenceControlModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.lstm = nn.LSTM(6, 32, batch_first=True)
        self.fc = nn.Linear(32, 2)
    
    def forward(self, x):
        # x shape: (batch, sequence_length, 6)
        _, (h, _) = self.lstm(x)
        return self.fc(h[-1])
```

## 性能优化

### 推理速度

- PyTorch: ~1-2ms (CPU), ~0.5ms (GPU)
- TensorFlow: ~2-3ms (CPU), ~1ms (GPU)

### 控制频率建议

- 实验室环境: 1-2 Hz
- 生产环境: 5-10 Hz
- 快速响应: 10-50 Hz (需要 GPU)

## 故障排查

### 问题 1: 模型加载失败

```
FileNotFoundError: 模型文件不存在
```

**解决**: 检查模型路径，使用绝对路径：
```yaml
path: "C:/ML/models/my_model.pth"
```

### 问题 2: 输入维度不匹配

```
RuntimeError: mat1 and mat2 shapes cannot be multiplied
```

**解决**: 确保配置的 `input_features` 数量与模型输入层一致。

### 问题 3: 输出不稳定

**可能原因**:
- 归一化参数不正确
- 模型训练不充分
- 安全限幅过于严格

**解决**:
1. 重新检查归一化参数
2. 使用更多数据重新训练
3. 调整 `max_voltage/max_current` 限制

## 与 PID 控制对比

| 特性 | PID 控制 | 深度学习控制 |
|------|---------|-------------|
| 参数调整 | 手动调参 | 自动学习 |
| 非线性 | 受限 | 优秀 |
| 多变量 | 困难 | 容易 |
| 可解释性 | 高 | 低 |
| 稳定性 | 保证 | 需验证 |
| 训练时间 | 无 | 需要 |

## 下一步

1. **收集数据**: 运行电解实验，记录传感器数据和控制命令
2. **训练模型**: 使用采集的数据训练神经网络
3. **验证模型**: 离线测试模型性能
4. **部署上线**: 替换配置文件中的模型路径
5. **持续优化**: 收集新数据，迭代改进模型

## 参考资料

- PyTorch 文档: https://pytorch.org/docs/
- TensorFlow 文档: https://tensorflow.org/docs/
- 强化学习控制: 考虑使用 RL 算法（DQN, PPO）

## 技术支持

如有问题，请查看日志输出或联系开发团队。

