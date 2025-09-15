

def register():
    # noinspection PyUnresolvedReferences
    from unilabos.devices.liquid_handling.prcxi.prcxi import PRCXI9300Deck
    # noinspection PyUnresolvedReferences
    from unilabos.devices.liquid_handling.prcxi.prcxi import PRCXI9300Container
    # noinspection PyUnresolvedReferences
    from unilabos.ros.nodes.presets.workstation import WorkStationContainer

    # 导入物料信息
    # from unilabos.devices.workstation.button_battery_station import * # type: ignore
    # 导入物料信息 - 具体导入所有类避免import *语法错误
    from unilabos.devices.workstation.coin_cell_assembly.button_battery_station import (
        ElectrodeSheetState, ElectrodeSheet, MaterialHoleState, MaterialHole,       
        MaterialPlateState, MaterialPlate, PlateSlot, ClipMagazineHole,
        ClipMagazine, BatteryState, Battery, BatteryPressSlotState,
        BatteryPressSlot, TipBox64State, TipBox64, WasteTipBoxstate,
        WasteTipBox, BottleRackState, BottleRack, BottleState, Bottle, CoincellDeck
    )
