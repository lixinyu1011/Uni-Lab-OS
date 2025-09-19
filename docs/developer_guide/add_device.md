# 添加新设备

在 Uni-Lab 中，设备（Device）是实验操作的基础单元。Uni-Lab 使用**注册表机制**来兼容管理种类繁多的设备驱动程序。回顾 {ref}`instructions` 中的概念，抽象的设备对外拥有【话题】【服务】【动作】三种通信机制，因此将设备添加进 Uni-Lab，实际上是将设备驱动中的三种机制映射到 Uni-Lab 标准指令集上。

能被 Uni-Lab 添加的驱动程序类型有以下种类：

1. Python Class，如

```python
class MockGripper:
    def __init__(self):
        self._position: float = 0.0
        self._velocity: float = 2.0
        self._torque: float = 0.0
        self._status = "Idle"

    @property
    def position(self) -> float:
        return self._position

    @property
    def velocity(self) -> float:
        return self._velocity

    @property
    def torque(self) -> float:
        return self._torque

    # 会被自动识别的设备属性，接入 Uni-Lab 时会定时对外广播
    @property
    def status(self) -> str:
        return self._status

    # 会被自动识别的设备动作，接入 Uni-Lab 时会作为 ActionServer 接受任意控制者的指令
    @status.setter
    def status(self, target):
        self._status = target

    # 需要在注册表添加的设备动作，接入 Uni-Lab 时会作为 ActionServer 接受任意控制者的指令
    def push_to(self, position: float, torque: float, velocity: float = 0.0):
        self._status = "Running"
        current_pos = self.position
        if velocity == 0.0:
            velocity = self.velocity

        move_time = abs(position - current_pos) / velocity
        for i in range(20):
            self._position = current_pos + (position - current_pos) / 20 * (i+1)
            self._torque = torque / (20 - i)
            self._velocity = velocity
            time.sleep(move_time / 20)
        self._torque = torque
        self._status = "Idle"
```

Python 类设备驱动在完成注册表后可以直接在 Uni-Lab 使用。

2. C# Class，如

```csharp
using System;
using System.Threading.Tasks;

public class MockGripper
{
    // 会被自动识别的设备属性，接入 Uni-Lab 时会定时对外广播
    public double position { get; private set; } = 0.0;
    public double velocity { get; private set; } = 2.0;
    public double torque { get; private set; } = 0.0;
    public string status { get; private set; } = "Idle";

    // 需要在注册表添加的设备动作，接入 Uni-Lab 时会作为 ActionServer 接受任意控制者的指令
    public async Task PushToAsync(double Position, double Torque, double Velocity = 0.0)
    {
        status = "Running";
        double currentPos = Position;
        if (Velocity == 0.0)
        {
            velocity = Velocity;
        }
        double moveTime = Math.Abs(Position - currentPos) / velocity;
        for (int i = 0; i < 20; i++)
        {
            position = currentPos + (Position - currentPos) / 20 * (i + 1);
            torque = Torque / (20 - i);
            velocity = Velocity;
            await Task.Delay((int)(moveTime * 1000 / 20)); // Convert seconds to milliseconds
        }
        torque = Torque;
        status = "Idle";
    }
}
```

C# 驱动设备在完成注册表后，需要调用 Uni-Lab C# 编译后才能使用，但只需一次。

## 快速开始：使用注册表编辑器（推荐）

推荐使用 Uni-Lab-OS 自带的可视化编辑器，它能自动分析您的设备驱动并生成大部分配置：

1. 启动 Uni-Lab-OS
2. 在浏览器中打开"注册表编辑器"页面
3. 选择您的 Python 设备驱动文件
4. 点击"分析文件"，让系统读取类信息
5. 填写基本信息（设备描述、图标等）
6. 点击"生成注册表"，复制生成的内容
7. 保存到 `devices/` 目录下

---

## 手动编写注册表（简化版）

如果需要手动编写，只需要提供两个必需字段，系统会自动补全其余内容：

### 最小配置示例

```yaml
my_device: # 设备唯一标识符
  class:
    module: unilabos.devices.your_module.my_device:MyDevice # Python 类路径
    type: python # 驱动类型
```

### 注册表文件位置

- 默认路径：`unilabos/registry/devices`
- 自定义路径：启动时使用 `--registry` 参数指定
- 可将多个设备写在同一个 yaml 文件中

### 系统自动生成的内容

系统会自动分析您的 Python 驱动类并生成：

- `status_types`：从 `get_*` 方法自动识别状态属性
- `action_value_mappings`：从类方法自动生成动作映射
- `init_param_schema`：从 `__init__` 方法分析初始化参数
- `schema`：前端显示用的属性类型定义

### 完整结构概览

```yaml
my_device:
  class:
    module: unilabos.devices.your_module.my_device:MyDevice
    type: python
    status_types: {} # 自动生成
    action_value_mappings: {} # 自动生成
  description: '' # 可选：设备描述
  icon: '' # 可选：设备图标
  init_param_schema: {} # 自动生成
  schema: {} # 自动生成
```

详细的注册表编写指南和高级配置，请参考{doc}`yaml 注册表编写指南 <add_yaml>`。
