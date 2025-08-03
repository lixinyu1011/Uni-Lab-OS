import collections

from pylabrobot.resources import opentrons_96_tiprack_10ul
from pylabrobot.resources.opentrons.plates import corning_96_wellplate_360ul_flat, nest_96_wellplate_2ml_deep

from unilabos.devices.liquid_handling.prcxi.prcxi import PRCXI9300Container, PRCXI9300Trash


def get_well_container(name: str) -> PRCXI9300Container:
    well_containers = corning_96_wellplate_360ul_flat(name).serialize()
    plate = PRCXI9300Container(name=name, size_x=50, size_y=50, size_z=10, category="plate",
                               ordering=collections.OrderedDict())
    plate_serialized = plate.serialize()
    well_containers.update({k: v for k, v in plate_serialized.items() if k not in ["children"]})
    new_plate: PRCXI9300Container = PRCXI9300Container.deserialize(well_containers)
    return new_plate

def get_tip_rack(name: str) -> PRCXI9300Container:
    tip_racks = opentrons_96_tiprack_10ul("name").serialize()
    tip_rack = PRCXI9300Container(name=name, size_x=50, size_y=50, size_z=10, category="tip_rack",
                       ordering=collections.OrderedDict())
    tip_rack_serialized = tip_rack.serialize()
    tip_racks.update({k: v for k, v in tip_rack_serialized.items() if k not in ["children"]})
    new_tip_rack: PRCXI9300Container = PRCXI9300Container.deserialize(tip_racks)
    return new_tip_rack

def prcxi_96_wellplate_360ul_flat(name: str):
    return get_well_container(name)

def prcxi_opentrons_96_tiprack_10ul(name: str):
    return get_tip_rack(name)

def prcxi_trash(name: str = None):
    return PRCXI9300Trash(name="trash", size_x=50, size_y=50, size_z=10, category="trash")

if __name__ == "__main__":
    # Example usage
    test_plate = prcxi_96_wellplate_360ul_flat("test_plate")
    test_rack = prcxi_opentrons_96_tiprack_10ul("test_rack")
    tash = prcxi_trash("trash")
    print(test_plate)
    print(test_rack)
    print(tash)
    # Output will be a dictionary representation of the PRCXI9300Container with well details