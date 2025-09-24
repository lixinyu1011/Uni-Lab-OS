from unilabos.resources.bottle_carrier import Bottle, BottleCarrier
# 工厂函数


def create_powder_bottle(
    name: str,
    diameter: float = 30.0,
    height: float = 50.0,
    max_volume: float = 50000.0,  # 50mL
) -> Bottle:
    """创建粉末瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        category="powder_bottle",
    )


def create_solution_beaker(
    name: str,
    diameter: float = 80.0,
    height: float = 100.0,
    max_volume: float = 500000.0,  # 500mL
) -> Bottle:
    """创建溶液烧杯"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        category="solution_beaker",
    )


def create_reagent_bottle(
    name: str,
    diameter: float = 20.0,
    height: float = 40.0,
    max_volume: float = 15000.0,  # 15mL
) -> Bottle:
    """创建试剂瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        category="reagent_bottle",
    )