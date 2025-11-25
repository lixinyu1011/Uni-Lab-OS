# -*- coding: utf-8 -*-
"""
电解水平台专用 Deck 配置
包含恒压源和恒流源作为物料资源
"""
from typing import Optional
from pylabrobot.resources import Deck, Coordinate, Container


class PowerSource(Container):
    """电源基类 - 恒压源和恒流源的基类"""
    
    def __init__(
        self,
        name: str,
        size_x: float = 100.0,
        size_y: float = 80.0,
        size_z: float = 50.0,
        max_voltage: float = 0.0,
        max_current: float = 0.0,
        category: str = "WE",
    ):
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            category=category,
        )
        self.max_voltage = max_voltage  # 最大电压 (V)
        self.max_current = max_current  # 最大电流 (mA)
        self.current_voltage = 0.0  # 当前电压设定 (V)
        self.current_current = 0.0  # 当前电流设定 (mA)
        self.is_active = False  # 是否激活


class ConstantVoltageSource(PowerSource):
    """恒压源 - 提供稳定电压输出"""
    
    def __init__(
        self,
        name: str,
        max_voltage: float = 5.0,  # 最大电压 5V
        max_current: float = 2000.0,  # 最大电流 2000mA
        size_x: float = 100.0,
        size_y: float = 80.0,
        size_z: float = 50.0,
        category="WE",
    ):
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            max_voltage=max_voltage,
            max_current=max_current,
            category=category,
        )
    
    def set_voltage(self, voltage: float):
        """设置输出电压"""
        if 0 <= voltage <= self.max_voltage:
            self.current_voltage = voltage
            return True
        return False
    
    def get_info(self) -> dict:
        """获取恒压源信息"""
        return {
            "name": self.name,
            "type": "恒压源",
            "max_voltage": f"{self.max_voltage}V",
            "max_current": f"{self.max_current}mA",
            "current_voltage": f"{self.current_voltage}V",
            "status": "激活" if self.is_active else "待机"
        }


class ConstantCurrentSource(PowerSource):
    """恒流源 - 提供稳定电流输出"""
    
    def __init__(
        self,
        name: str,
        max_voltage: float = 5.0,  # 最大电压 5V
        max_current: float = 2000.0,  # 最大电流 2000mA
        size_x: float = 100.0,
        size_y: float = 80.0,
        size_z: float = 50.0,
        category="WE",
    ):
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            max_voltage=max_voltage,
            max_current=max_current,
            category=category,
        )
    
    def set_current(self, current: float):
        """设置输出电流"""
        if 0 <= current <= self.max_current:
            self.current_current = current
            return True
        return False
    
    def get_info(self) -> dict:
        """获取恒流源信息"""
        return {
            "name": self.name,
            "type": "恒流源",
            "max_voltage": f"{self.max_voltage}V",
            "max_current": f"{self.max_current}mA",
            "current_current": f"{self.current_current}mA",
            "status": "激活" if self.is_active else "待机"
        }


class ElectrolysisReactor(Container):
    """电解反应器 - 实际进行电解反应的容器"""
    
    def __init__(
        self,
        name: str,
        volume: float = 100.0,  # 容量 (mL)
        size_x: float = 60.0,
        size_y: float = 60.0,
        size_z: float = 100.0,
    ):
        super().__init__(
            name=name,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            category="WE",
        )
        self.volume = volume  # 反应器容量 (mL)
        self.current_volume = 0.0  # 当前液体体积 (mL)
        self.temperature = 25.0  # 当前温度 (°C)
        self.pH = 7.0  # 当前pH值
    
    def get_info(self) -> dict:
        """获取反应器信息"""
        return {
            "name": self.name,
            "type": "电解反应器",
            "volume": f"{self.volume}mL",
            "current_volume": f"{self.current_volume}mL",
            "temperature": f"{self.temperature}°C",
            "pH": self.pH
        }


class ElectrolysisDeck(Deck):
    """电解水平台专用 Deck 类"""
    
    def __init__(
        self,
        name: str = "electrolysis_deck",
        size_x: float = 800.0,
        size_y: float = 600.0,
        size_z: float = 100.0,
        category: str = "WE",
        setup: bool = False,
    ) -> None:
        """
        初始化电解水平台 Deck
        
        Args:
            name: Deck 名称
            size_x: Deck 长度 (mm)
            size_y: Deck 宽度 (mm)
            size_z: Deck 高度 (mm)
            category: 类别
            setup: 是否自动执行 setup
            **kwargs: 其他参数（会被自动过滤，避免传入 Deck 不支持的参数）
        """
        # 过滤掉 Deck 基类不支持的参数
        # 这些参数可能来自配置文件，但 PyLabRobot 的 Deck 不接受
        
        super().__init__(name=name, size_x=size_x, size_y=size_y, size_z=size_z, category=category)
        
        # 存储电源和反应器
        self.power_sources = {}
        self.reactors = {}
        
        if setup:
            self.setup()
    
    def setup(self) -> None:
        """配置电解水平台的所有资源"""
        # 添加电源（恒压源和恒流源）
        self.power_sources = {
            "恒压源-1": ConstantVoltageSource(
                name="voltage_source_1",
                max_voltage=5.0,      # 最大5V
                max_current=2000.0,   # 最大2000mA
            ),
            "恒流源-1": ConstantCurrentSource(
                name="current_source_1",
                max_voltage=5.0,      # 最大5V
                max_current=2000.0,   # 最大2000mA
            ),
        }
        
        # 添加反应器
        self.reactors = {
            "反应器-1": ElectrolysisReactor(
                name="reactor_1",
                volume=100.0,  # 100mL容量
            ),
            "反应器-2": ElectrolysisReactor(
                name="reactor_2",
                volume=100.0,  # 100mL容量
            ),
        }
        
        # 电源位置配置（放在 Deck 中间，z=50 露出名称）
        deck_center_x = self.get_size_x() / 2
        deck_center_y = self.get_size_y() / 2
        
        self.power_source_locations = {
            "恒压源-1": Coordinate(deck_center_x - 100.0, deck_center_y, 50.0),   # 中间偏左，高50mm
            "恒流源-1": Coordinate(deck_center_x + 100.0, deck_center_y, 50.0),   # 中间偏右，高50mm
        }
        
        # 反应器位置配置（围绕中间放置，z=50 露出名称）
        self.reactor_locations = {
            "反应器-1": Coordinate(deck_center_x - 150.0, deck_center_y + 150.0, 50.0),   # 左下，高50mm
            "反应器-2": Coordinate(deck_center_x + 150.0, deck_center_y + 150.0, 50.0),   # 右下，高50mm
        }
        
        # 将电源分配到 Deck 上
        for power_source_name, power_source in self.power_sources.items():
            self.assign_child_resource(
                power_source,
                location=self.power_source_locations[power_source_name]
            )
        
        # 将反应器分配到 Deck 上
        for reactor_name, reactor in self.reactors.items():
            self.assign_child_resource(
                reactor,
                location=self.reactor_locations[reactor_name]
            )


def create_electrolysis_deck(
    deck_name: str = "electrolysis_deck",
    size_x: float = 800.0,
    size_y: float = 600.0,
    size_z: float = 100.0,
    setup: bool = True
) -> ElectrolysisDeck:
    """
    创建电解水平台专用 Deck（工厂函数）
    
    Args:
        deck_name: Deck 名称
        size_x: Deck 长度 (mm)
        size_y: Deck 宽度 (mm)
        size_z: Deck 高度 (mm)
        setup: 是否自动配置资源
    
    Returns:
        配置好的 ElectrolysisDeck 实例
    """
    deck = ElectrolysisDeck(
        name=deck_name,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        setup=setup
    )
    return deck


def print_deck_layout(deck: Deck):
    """打印 Deck 布局信息"""
    print(f"\n{'='*60}")
    print(f"电解水平台 Deck 布局")
    print(f"{'='*60}")
    print(f"Deck 名称: {deck.name}")
    print(f"Deck 尺寸: {deck.get_size_x()}mm × {deck.get_size_y()}mm × {deck.get_size_z()}mm")
    print(f"\n物料清单 ({len(deck.children)} 个):")
    print(f"{'-'*60}")
    
    for i, child in enumerate(deck.children, 1):
        location = child.location
        print(f"\n{i}. {child.name}")
        print(f"   类型: {child.category}")
        print(f"   尺寸: {child.get_size_x()}mm × {child.get_size_y()}mm × {child.get_size_z()}mm")
        print(f"   位置: X={location.x:.1f}mm, Y={location.y:.1f}mm, Z={location.z:.1f}mm")
        
        # 显示详细信息
        if hasattr(child, 'get_info'):
            info = child.get_info()
            for key, value in info.items():
                if key not in ['name', 'type']:
                    print(f"   {key}: {value}")
    
    print(f"\n{'='*60}\n")


# 示例用法
if __name__ == "__main__":
    print("="*60)
    print("电解水平台 Deck 配置示例")
    print("="*60)
    
    # 方法 1: 使用工厂函数创建
    print("\n方法 1: 使用 create_electrolysis_deck() 函数")
    deck1 = create_electrolysis_deck()
    print_deck_layout(deck1)
    
    # 方法 2: 直接使用类创建
    print("\n方法 2: 直接使用 ElectrolysisDeck 类")
    deck2 = ElectrolysisDeck(
        name="my_electrolysis_deck",
        size_x=1200.0,
        size_y=900.0,
        size_z=150.0,
        setup=True
    )
    
    # 访问和操作资源
    print("\n演示资源操作:")
    print("-" * 60)
    
    # 通过字典访问电源
    if "恒压源-1" in deck2.power_sources:
        voltage_source = deck2.power_sources["恒压源-1"]
        voltage_source.set_voltage(3.5)
        voltage_source.is_active = True
        print(f"✓ 设置恒压源电压: {voltage_source.current_voltage}V")
    
    # 通过 get_resource 访问
    current_source = deck2.get_resource("current_source_1")
    if isinstance(current_source, ConstantCurrentSource):
        current_source.set_current(1500.0)
        current_source.is_active = True
        print(f"✓ 设置恒流源电流: {current_source.current_current}mA")
    
    # 通过字典访问反应器
    if "反应器-1" in deck2.reactors:
        reactor = deck2.reactors["反应器-1"]
        reactor.current_volume = 80.0
        reactor.temperature = 30.0
        reactor.pH = 6.5
        print(f"✓ 反应器状态: {reactor.current_volume}mL, {reactor.temperature}°C, pH={reactor.pH}")
    
    # 列出所有资源
    print(f"\n✓ Deck 共有 {len(deck2.children)} 个子资源:")
    print(f"  - 电源: {list(deck2.power_sources.keys())}")
    print(f"  - 反应器: {list(deck2.reactors.keys())}")
    
    print("\n" + "="*60)
    print("电解水平台 Deck 配置完成！")
    print("="*60)

