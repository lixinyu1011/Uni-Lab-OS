# 电池装配工站接入（PLC）

本指南将引导你完成电池装配工站（以 PLC 控制为例）的接入流程，包括新建工站文件、编写驱动与寄存器读写、生成注册表、上传并联调。

## 1. 新建工站文件

### 1.1 创建工站文件

在 `unilabos/devices/workstation/coin_cell_assembly/` 目录下新建工站文件，如 `coin_cell_assembly.py`。工站类需继承 `WorkstationBase`，并在构造函数中初始化通信客户端与寄存器映射。

```python
from typing import Optional
# 工站基类
from unilabos.devices.workstation.workstation_base import WorkstationBase
# Modbus 通讯与寄存器 CSV 支持
from unilabos.device_comms.modbus_plc.client import TCPClient, BaseClient

class CoinCellAssemblyWorkstation(WorkstationBase):
    def __init__(
        self,
        station_resource,
        address: str = "192.168.1.20",
        port: str = "502",
        *args,
        **kwargs,
    ):
        super().__init__(station_resource=station_resource, *args, **kwargs)
        self.station_resource = station_resource  # 物料台面（Deck）
        self.success: bool = False
        self.allow_data_read: bool = False
        self.csv_export_thread = None
        self.csv_export_running = False
        self.csv_export_file: Optional[str] = None

        # 连接 PLC，并注册寄存器节点
        tcp = TCPClient(addr=address, port=port)
        tcp.client.connect()
        self.nodes = BaseClient.load_csv(".../PLC_register.csv")
        self.client = tcp.register_node_list(self.nodes)
```

### 1.2 约定与命名

建议将寄存器按“命令（COIL_*）/状态（*_STATUS）/数据（REG_*）”分层命名，保持 CSV 与代码一致，便于维护与测试。

## 2. 编写驱动与寄存器读写

### 2.1 寄存器示例

- `COIL_SYS_START_CMD`（BOOL，地址 8010）：启动命令（脉冲式）
- `COIL_SYS_START_STATUS`（BOOL，地址 8210）：启动状态
- `REG_DATA_OPEN_CIRCUIT_VOLTAGE`（FLOAT32，地址 10002）：开路电压
- `REG_DATA_ASSEMBLY_PRESSURE`（INT16，地址 10014）：压制扣电压力

### 2.2 最小驱动示例

```python
from unilabos.device_comms.modbus_plc.modbus import WorderOrder

def start_and_read_metrics(self):
    # 1) 下发启动（置 True 再复位 False）
    self.client.use_node('COIL_SYS_START_CMD').write(True)
    self.client.use_node('COIL_SYS_START_CMD').write(False)

    # 2) 等待进入启动状态
    while True:
        status, _ = self.client.use_node('COIL_SYS_START_STATUS').read(1)
        if bool(status[0]):
            break

    # 3) 读取关键数据（FLOAT32 需读 2 个寄存器并指定字节序）
    voltage, _ = self.client.use_node('REG_DATA_OPEN_CIRCUIT_VOLTAGE').read(
        2, word_order=WorderOrder.LITTLE
    )
    pressure, _ = self.client.use_node('REG_DATA_ASSEMBLY_PRESSURE').read(1)

    return {
        'open_circuit_voltage': voltage,
        'assembly_pressure': pressure,
    }
```

> 提示：若需参数下发，可在 PLC 端设置标志寄存器并完成握手复位，避免粘连与竞争。

## 3. 本地生成注册表并校验

完成工站类与驱动后，需要生成（或更新）工站注册表供系统识别。

### 3.1 生成注册表

```bash
python unilabos\app\register.py --config cellconfig.py --complete_registry
```

### 3.2 校验要点

- 在生成的 YAML 中，确认 `module` 指向新工站类，例如：

```
module: unilabos.devices.workstation.coin_cell_assembly.coin_cell_assembly:CoinCellAssemblyWorkstation
```

- 每次新增/删除导出的驱动能力后，需重新生成注册表。

## 4. 上传并联调（云端）

### 4.1 启动并上传注册表

```bash
python unilabos\app\main.py -g celljson.json --ak <user的AK> --sk <user的SK> --websocket --upload_registry
```

参数说明：
- `-g/--graph`：节点/物料图 JSON（如 `celljson.json`）
- `--upload_registry`：同步上传新工站 Python 与注册表
- `--websocket`：启用与云端的长连接通信

## 5. 提交 Pull Request（可选）

若你的工站将贡献到主仓库，建议以以下流程提交：

1. 从 `main` 创建分支（如：`feat/add-battery-plc-station`）
2. 提交并推送改动后创建 Pull Request，描述：
   - 工站功能概述、关键寄存器说明
   - 本地/云端联调结果与注意事项

## 6. 常见问题（FAQ）

**Q: 启动命令写入后没有生效？**  
A: 确认写入后复位 False，并检查启动状态位地址是否正确。

**Q: 读取浮点数据异常？**  
A: 确保 `FLOAT32` 读取长度为 2 并指定 `word_order`；不同 PLC 字节序可能不同。

**Q: 注册表不识别我的工站？**  
A: 检查 YAML 的 `module` 路径与类名是否匹配；修改驱动后重新生成注册表。

**Q: 是否每次修改驱动都要上传？**  
A: 是。新增/删除导出的驱动能力后，需重新生成并上传最新注册表。
