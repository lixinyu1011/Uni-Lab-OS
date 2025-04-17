# yaml注册表编写指南

`注册表的结构`

1. 顶层名称：每个设备的注册表以设备名称开头，例如 new_device。
2. class 字段：定义设备的模块路径和类型。
3. schema 字段：定义设备的属性模式，包括属性类型、描述和必需字段。
4. action_value_mappings 字段：定义设备支持的动作及其目标、反馈和结果。

`创建新的注册表教程`
1. 创建文件
    在 devices 文件夹中创建一个新的 YAML 文件，例如 new_device.yaml。

2. 定义设备名称
    在文件中定义设备的顶层名称，例如：new_device

3. 定义设备的类信息
    添加设备的模块路径和类型：

```python
new_device:  # 定义一个名为 linear_motion.grbl 的设备


class:  # 定义设备的类信息
    module: unilabos.devices_names.new_device:NewDeviceClass  # 指定模块路径和类名
    type: python  # 指定类型为 Python 类
    status_types:
```
4. 定义设备支持的动作
    添加设备支持的动作及其目标、反馈和结果：
```python
  action_value_mappings:
    set_speed:
      type: SendCmd
      goal:
        command: speed
      feedback: {}
      result:
        success: success
```
`如何编写action_valve_mappings`
1. 在 devices 文件夹中的 YAML 文件中，action_value_mappings 是用来定义设备支持的动作（actions）及其目标值（goal）、反馈值（feedback）和结果值（result）的映射规则。以下是规则和编写方法：
```python
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
                                        # 根据动作的功能选择合适的类型：
                                        # SendCmd：发送简单命令。
                                        # NavigateThroughPoses：导航动作。
                                        # SingleJointPosition：设置单一关节的位置。
                                        # Stir：搅拌动作。
                                        # HeatChill：加热或冷却动作。

      goal:                             # 定义动作的目标值映射，表示需要传递给设备的参数。
        <goal_key>: <mapped_value>      #确定设备需要的输入参数，并将其映射到设备的字段。

      feedback:                         # 定义动作的反馈值映射，表示设备执行动作时返回的实时状态。
        <feedback_key>: <mapped_value>
      result:                           # 定义动作的结果值映射，表示动作完成后返回的最终结果。
        <result_key>: <mapped_value>
```

6. 定义设备的属性模式
    添加设备的属性模式，包括属性类型和描述：
```python
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
# 写完yaml注册表后需要添加到哪些其他文件？
