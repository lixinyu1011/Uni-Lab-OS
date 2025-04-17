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

## 注册表文件位置

Uni-Lab 启动时会自动读取默认注册表路径 `unilabos/registry/devices` 下的所有注册设备。您也可以任意维护自己的注册表路径，只需要在 Uni-Lab 启动时使用 `--registry` 参数将路径添加即可。

在 `<path-to-registry>/devices` 中新建一个 yaml 文件，即可开始撰写。您可以将多个设备写到同一个 yaml 文件中。

## 注册表的结构

1. 顶层名称：每个设备的注册表以设备名称开头，例如 `new_device`, `gripper.mock`。
1. `class` 字段：定义设备的模块路径和驱动程序语言。
1. `status_types` 字段：定义设备定时对 Uni-Lab 实验室内发送的属性名及其类型。
1. `action_value_mappings` 字段：定义设备支持的动作及其目标、反馈和结果。
1. `schema` 字段：定义设备定时对 Uni-Lab 云端监控发送的属性名及其类型、描述（非必须）

## 创建新的注册表教程

1. 创建文件
   在 devices 文件夹中创建一个新的 YAML 文件，例如 `new_device.yaml`。
2. 定义设备名称
   在文件中定义设备的顶层名称，例如：`new_device` 或 `gripper.mock`
3. 定义设备的类信息
   添加设备的模块路径和类型：

```yaml
gripper.mock:
  class:   # 定义设备的类信息
    module: unilabos.devices.gripper.mock:MockGripper
    type: python  # 指定驱动语言为 Python
    status_types:
      position: Float64
      torque: Float64
      status: String
```

4. 定义设备的定时发布属性。注意，对于 Python Class 来说，PROP 是 class 的 `property`，或满足能被 `getattr(cls, PROP)` 或 `cls.get_PROP` 读取到的属性值的对象。

```yaml
    status_types:
      PROP: TYPE
```
5. 定义设备支持的动作
   添加设备支持的动作及其目标、反馈和结果：

```yaml
    action_value_mappings:
      set_speed:
        type: SendCmd
        goal:
          command: speed
        feedback: {}
        result:
          success: success
```

在 devices 文件夹中的 YAML 文件中，action_value_mappings 是用来将驱动内的动作函数，映射到 Uni-Lab 标准动作（actions）及其目标参数值（goal）、反馈值（feedback）和结果值（result）的映射规则。若在 Uni-Lab 指令集内找不到符合心意的，请【创建新指令】。

```yaml
  action_value_mappings:
    <action_name>:                      # <action_name>：动作的名称
                                        # start：启动设备或某个功能。
                                        # stop：停止设备或某个功能。
                                        # set_speed：设置设备的速度。
                                        # set_temperature：设置设备的温度。
                                        # move_to_position：移动设备到指定位置。
                                        # stir：执行搅拌操作。
                                        # heatchill：执行加热或冷却操作。
                                        # send_nav_task：发送导航任务（例如机器人导航）。
                                        # set_timer：设置设备的计时器。
                                        # valve_open_cmd：打开阀门。
                                        # valve_close_cmd：关闭阀门。
                                        # execute_command_from_outer：执行外部命令。
                                        # push_to：控制设备推送到某个位置（例如机械爪）。
                                        # move_through_points：导航设备通过多个点。

      type: <ActionType>                # 动作的类型，表示动作的功能
                                        # 根据动作的功能选择合适的类型，请查阅 Uni-Lab 已支持的指令集。

      goal:                             # 定义动作的目标值映射，表示需要传递给设备的参数。
        <goal_key>: <mapped_value>      #确定设备需要的输入参数，并将其映射到设备的字段。

      feedback:                         # 定义动作的反馈值映射，表示设备执行动作时返回的实时状态。
        <feedback_key>: <mapped_value>
      result:                           # 定义动作的结果值映射，表示动作完成后返回的最终结果。
        <result_key>: <mapped_value>
```

6. 定义设备的网页展示属性类型，这部分会被用于在 Uni-Lab 网页端渲染成状态监控
   添加设备的属性模式，包括属性类型和描述：

```yaml
schema:
    type: object
    properties:
      status:
        type: string
        description: The status of the device
      speed:
        type: number
        description: The speed of the device
    required:
      - status
      - speed
    additionalProperties: false
```
