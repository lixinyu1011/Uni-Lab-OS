from unilabos.devices.workstation.ResinWorkstation.ResinWorkstation_warehouse import WareHouse, warehouse_factory



# =================== Other ===================


def Hydrogel_warehouse_1x5x1(name: str) -> WareHouse:
    """创建水凝胶模块 1x5x1仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=5,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        letter_replace={"A": "R", "C": "F"},
    )

def Station_1_warehouse_4x2x1(name: str) -> WareHouse:
    """创建反应工站1仓库（4列 x 2行，共8个槽位）"""
    return warehouse_factory(
        name=name,
        num_items_x=4,
        num_items_y=2,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        naming_mode="continuous_number",   # 使用连续数字
        name_prefix="S1_",                 # 前缀，生成 S1_1, S1_2, ..., S1_8
    )

def Station_2_warehouse_1x1x1(name: str) -> WareHouse:
    """创建检测工站 1x1x1仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=1,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        custom_keys=["Station_2"],  # 使用数字2作为编号
    )

def Station_3_warehouse_1x1x1(name: str) -> WareHouse:
    """创建检测工站 1x1x1仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=1,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        custom_keys=["Station_3"],  # 使用数字3作为编号
    )


def Raw_electrode_warehouse_3x5x1(name: str) -> WareHouse:
    """创建原始电极 3x5x1仓库（序号从右往左：右=R1, 中=R2, 左=R3）"""
    return warehouse_factory(
        name=name,
        num_items_x=3,
        num_items_y=5,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        naming_mode="continuous_number",
        reverse_col_order=True,
        name_prefix="R",
    )

def Finished_electrode_warehouse_3x5x1(name: str) -> WareHouse:
    """创建完成电极 3x5x1仓库（序号从右往左：右=F1, 中=F2, 左=F3）"""
    return warehouse_factory(
        name=name,
        num_items_x=3,
        num_items_y=5,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        naming_mode="continuous_number",
        reverse_col_order=True,
        name_prefix="F",
    )

def Stir_1_warehouse_1x1x1(name: str) -> WareHouse:
    """创建搅拌仪 1x1x1仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=1,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        custom_keys=["Test_1"],  # 使用数字0作为编号
    )

def Stir_2_warehouse_1x1x1(name: str) -> WareHouse:
    """创建搅拌仪 1x1x1仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=1,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        custom_keys=["Test_2"],  # 使用数字0作为编号
    )

def Water_wash_warehouse_1x1x1(name: str) -> WareHouse:
    """创建水洗 1x1x1仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=1,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        custom_keys=["Wash"],  # 使用数字0作为编号
    )

def Acid_wash_warehouse_1x1x1(name: str) -> WareHouse:
    """创建酸洗 1x1x1仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=1,
        num_items_y=1,
        num_items_z=1,
        dx=10.0,
        dy=10.0,
        dz=10.0,
        item_dx=137.0,
        item_dy=96.0,
        item_dz=120.0,
        category="warehouse",
        custom_keys=["Acid"],  # 使用数字0作为编号
    )