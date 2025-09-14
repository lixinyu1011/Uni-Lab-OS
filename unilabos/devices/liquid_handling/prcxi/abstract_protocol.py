import asyncio
import collections
import contextlib
import json
import socket
import time
from typing import Any, List, Dict, Optional, TypedDict, Union, Sequence, Iterator, Literal

from pylabrobot.liquid_handling import (
    LiquidHandlerBackend,
    Pickup,
    SingleChannelAspiration,
    Drop,
    SingleChannelDispense,
    PickupTipRack,
    DropTipRack,
    MultiHeadAspirationPlate, ChatterBoxBackend, LiquidHandlerChatterboxBackend,
)
from pylabrobot.liquid_handling.standard import (
    MultiHeadAspirationContainer,
    MultiHeadDispenseContainer,
    MultiHeadDispensePlate,
    ResourcePickup,
    ResourceMove,
    ResourceDrop,
)
from pylabrobot.resources import Tip, Deck, Plate, Well, TipRack, Resource, Container, Coordinate, TipSpot, Trash

from unilabos.devices.liquid_handling.liquid_handler_abstract import LiquidHandlerAbstract




class MaterialResource:
    """统一的液体/反应器资源，支持多孔（wells）场景：
    - wells: 列表，每个元素代表一个物料孔（unit）；
    - units: 与 wells 对齐的列表，每个元素是 {liquid_id: volume}；
    - 若传入 liquid_id + volume 或 composition，总量将**等分**到各 unit；
    """
    def __init__(
        self,
        resource_name: str,
        slot: int,
        well: List[int],
        composition: Optional[Dict[str, float]] = None,
        liquid_id: Optional[str] = None,
        volume: Union[float, int] = 0.0,
        is_supply: Optional[bool] = None,
    ):
        self.resource_name = resource_name
        self.slot = int(slot)
        self.well = list(well or [])
        self.is_supply = bool(is_supply) if is_supply is not None else (bool(composition) or (liquid_id is not None))

        # 规范化：至少有 1 个 unit
        n = max(1, len(self.well))
        self.units: List[Dict[str, float]] = [dict() for _ in range(n)]

        # 初始化内容：等分到各 unit
        if composition:
            for k, v in composition.items():
                share = float(v) / n
                for u in self.units:
                    if share > 0:
                        u[k] = u.get(k, 0.0) + share
        elif liquid_id is not None and float(volume) > 0:
            share = float(volume) / n
            for u in self.units:
                u[liquid_id] = u.get(liquid_id, 0.0) + share

    # 位置描述
    def location(self) -> Dict[str, Any]:
        return {"slot": self.slot, "well": self.well}

    def unit_count(self) -> int:
        return len(self.units)

    def unit_volume(self, idx: int) -> float:
        return float(sum(self.units[idx].values()))

    def total_volume(self) -> float:
        return float(sum(self.unit_volume(i) for i in range(self.unit_count())))

    def add_to_unit(self, idx: int, liquid_id: str, vol: Union[float, int]):
        v = float(vol)
        if v < 0:
            return
        u = self.units[idx]
        if liquid_id not in u:
           u[liquid_id] = 0.0
        if v > 0:
           u[liquid_id] += v

    def remove_from_unit(self, idx: int, total: Union[float, int]) -> Dict[str, float]:
        take = float(total)
        if take <= 0: return {}
        u = self.units[idx]
        avail = sum(u.values())
        if avail <= 0: return {}
        take = min(take, avail)
        ratio = take / avail
        removed: Dict[str, float] = {}
        for k, v in list(u.items()):
            dv = v * ratio
            nv = v - dv
            if nv < 1e-9: nv = 0.0
            u[k] = nv
            removed[k] = dv

        self.units[idx] = {k: v for k, v in u.items() if v > 0}
        return removed

    def transfer_unit_to(self, src_idx: int, other: "MaterialResource", dst_idx: int, total: Union[float, int]):
        moved = self.remove_from_unit(src_idx, total)
        for k, v in moved.items():
            other.add_to_unit(dst_idx, k, v)

    def get_resource(self) -> Dict[str, Any]:
        return {
            "resource_name": self.resource_name,
            "slot": self.slot,
            "well": self.well,
            "units": [dict(u) for u in self.units],
            "total_volume": self.total_volume(),
            "is_supply": self.is_supply,
        }

def transfer_liquid(
    sources: MaterialResource,
    targets: MaterialResource,
    unit_volume: Optional[Union[float, int]] = None,
    tip: Optional[str] = None, #这里应该是指定种类的
) -> Dict[str, Any]:
    try:
        vol_each = float(unit_volume)
    except (TypeError, ValueError):
        return {"action": "transfer_liquid", "error": "invalid unit_volume"}
    if vol_each <= 0:
        return {"action": "transfer_liquid", "error": "non-positive volume"}

    ns, nt = sources.unit_count(), targets.unit_count()
    # one-to-many: 从单个 source unit(0) 扇出到目标各 unit
    if ns == 1 and nt >= 1:
        for j in range(nt):
            sources.transfer_unit_to(0, targets, j, vol_each)
    # many-to-many: 数量相同，逐一对应
    elif ns == nt and ns > 0:
        for i in range(ns):
            sources.transfer_unit_to(i, targets, i, vol_each)
    else:
        raise ValueError(f"Unsupported mapping: sources={ns} units, targets={nt} units. Only 1->N or N->N are allowed.")

    return {
        "action": "transfer_liquid",
        "sources": sources.get_resource(),
        "targets": targets.get_resource(),
        "unit_volume": unit_volume,
        "tip": tip, 
    }

def plan_transfer(pm: "ProtocolManager", **kwargs) -> Dict[str, Any]:
    """Shorthand to add a non-committing transfer to a ProtocolManager.
    Accepts the same kwargs as ProtocolManager.add_transfer.
    """
    return pm.add_transfer(**kwargs)

class ProtocolManager:
    """Plan/track transfers and back‑solve minimum initial volumes.

    Use add_transfer(...) to register steps (no mutation).
    Use compute_min_initials(...) to infer the minimal starting volume of each liquid
    per resource required to execute the plan in order.
    """

    # ---------- lifecycle ----------
    def __init__(self):
        # queued logical steps (keep live refs to MaterialResource)
        self.steps: List[Dict[str, Any]] = []

        # simple tip catalog; choose the smallest that meets min_aspirate and capacity*safety
        self.tip_catalog = [
            {"name": "TIP_10uL",   "capacity": 10.0,   "min_aspirate": 0.5},
            {"name": "TIP_20uL",   "capacity": 20.0,   "min_aspirate": 1.0},
            {"name": "TIP_50uL",   "capacity": 50.0,   "min_aspirate": 2.0},
            {"name": "TIP_200uL",  "capacity": 200.0,  "min_aspirate": 5.0},
            {"name": "TIP_300uL",  "capacity": 300.0,  "min_aspirate": 10.0},
            {"name": "TIP_1000uL", "capacity": 1000.0, "min_aspirate": 20.0},
        ]

        # stable labels for unknown liquids per resource (A, B, C, ..., AA, AB, ...)
        self._unknown_labels: Dict[MaterialResource, str] = {}
        self._unknown_label_counter: int = 0

    # ---------- public API ----------
    def recommend_tip(self, unit_volume: float, safety: float = 1.10) -> str:
        v = float(unit_volume)
        # prefer: meets min_aspirate and capacity with safety margin; else fallback to capacity-only; else max capacity
        eligible = [t for t in self.tip_catalog if t["min_aspirate"] <= v and t["capacity"] >= v * safety]
        if not eligible:
            eligible = [t for t in self.tip_catalog if t["capacity"] >= v]
        return min(eligible or self.tip_catalog, key=lambda t: t["capacity"]) ["name"]

    def get_tip_capacity(self, tip_name: str) -> Optional[float]:
        for t in self.tip_catalog:
            if t["name"] == tip_name:
                return t["capacity"]
        return None

    def add_transfer(
        self,
        sources: MaterialResource,
        targets: MaterialResource,
        unit_volume: Union[float, int],
        tip: Optional[str] = None,
    ) -> Dict[str, Any]:
        step = {
            "action": "transfer_liquid",
            "sources": sources,
            "targets": targets,
            "unit_volume": float(unit_volume),
            "tip": tip or self.recommend_tip(unit_volume),
        }
        self.steps.append(step)
        # return a serializable shadow (no mutation)
        return {
            "action": "transfer_liquid",
            "sources": sources.get_resource(),
            "targets": targets.get_resource(),
            "unit_volume": step["unit_volume"],
            "tip": step["tip"],
        }

    @staticmethod
    def _liquid_keys_of(resource: MaterialResource) -> List[str]:
        keys: set[str] = set()
        for u in resource.units:
            keys.update(u.keys())
        return sorted(keys)

    @staticmethod
    def _fanout_multiplier(ns: int, nt: int) -> Optional[int]:
        """Return the number of liquid movements for a mapping shape.
        1->N: N moves; N->N: N moves; otherwise unsupported (None).
        """
        if ns == 1 and nt >= 1:
            return nt
        if ns == nt and ns > 0:
            return ns
        return None

    # ---------- planning core ----------
    def compute_min_initials(
        self,
        use_initial: bool = False,
        external_only: bool = True,
    ) -> Dict[str, Dict[str, float]]:
        """Simulate the plan (non‑mutating) and return minimal starting volumes per resource/liquid."""
        ledger: Dict[MaterialResource, Dict[str, float]] = {}
        min_seen: Dict[MaterialResource, Dict[str, float]] = {}

        def _ensure(res: MaterialResource) -> None:
            if res in ledger:
                return
            declared = self._liquid_keys_of(res)
            if use_initial:
                # sum actual held amounts across units
                totals = {k: 0.0 for k in declared}
                for u in res.units:
                    for k, v in u.items():
                        totals[k] = totals.get(k, 0.0) + float(v)
                ledger[res] = totals
            else:
                ledger[res] = {k: 0.0 for k in declared}
            min_seen[res] = {k: ledger[res].get(k, 0.0) for k in ledger[res]}

        def _proportions(src: MaterialResource, src_bal: Dict[str, float]) -> tuple[List[str], Dict[str, float]]:
            keys = list(src_bal.keys())
            total_pos = sum(x for x in src_bal.values() if x > 0)

            # if ledger has no keys yet, seed from declared types on the resource
            if not keys:
                keys = self._liquid_keys_of(src)
                for k in keys:
                    src_bal.setdefault(k, 0.0)
                    min_seen[src].setdefault(k, 0.0)

            if total_pos > 0:
                # proportional to current positive balances
                props = {k: (src_bal.get(k, 0.0) / total_pos) for k in keys}
                return keys, props

            # no material currently: evenly from known keys, or assign an unknown label
            if keys:
                eq = 1.0 / len(keys)
                return keys, {k: eq for k in keys}

            unk = self._label_for_unknown(src)
            keys = [unk]
            src_bal.setdefault(unk, 0.0)
            min_seen[src].setdefault(unk, 0.0)
            return keys, {unk: 1.0}

        for step in self.steps:
            if step.get("action") != "transfer_liquid":
                continue

            src: MaterialResource = step["sources"]
            dst: MaterialResource = step["targets"]
            vol = float(step["unit_volume"])
            if vol <= 0:
                continue

            _ensure(src)
            _ensure(dst)

            mult = self._fanout_multiplier(src.unit_count(), dst.unit_count())
            if not mult:
                continue  # unsupported mapping shape for this planner

            eff_vol = vol * mult
            src_bal = ledger[src]
            keys, props = _proportions(src, src_bal)

            # subtract from src; track minima; accumulate to dst
            moved: Dict[str, float] = {}
            for k in keys:
                dv = eff_vol * props[k]
                src_bal[k] = src_bal.get(k, 0.0) - dv
                moved[k] = dv
                prev_min = min_seen[src].get(k, 0.0)
                if src_bal[k] < prev_min:
                    min_seen[src][k] = src_bal[k]

            dst_bal = ledger[dst]
            for k, dv in moved.items():
                dst_bal[k] = dst_bal.get(k, 0.0) + dv
                min_seen[dst].setdefault(k, dst_bal[k])

        # convert minima (negative) to required initials
        result: Dict[str, Dict[str, float]] = {}
        for res, mins in min_seen.items():
            if external_only and not getattr(res, "is_supply", False):
                continue
            need = {liq: max(0.0, -mn) for liq, mn in mins.items() if mn < 0.0}
            if need:
                result[res.resource_name] = need
        return result

    def compute_tip_consumption(self) -> Dict[str, Any]:
        """Compute how many tips are consumed at each transfer step, and aggregate by tip type.
        Rule: each liquid movement (source unit -> target unit) consumes one tip.
        For supported shapes: 1->N uses N tips; N->N uses N tips.
        """
        per_step: List[Dict[str, Any]] = []
        totals_by_tip: Dict[str, int] = {}

        for i, s in enumerate(self.steps):
            if s.get("action") != "transfer_liquid":
                continue
            ns = s["sources"].unit_count()
            nt = s["targets"].unit_count()
            moves = self._fanout_multiplier(ns, nt) or 0
            tip_name = s.get("tip") or self.recommend_tip(s["unit_volume"])  # per-step tip may vary
            per_step.append({
                "idx": i,
                "tip": tip_name,
                "tips_used": moves,
                "moves": moves,
            })
            totals_by_tip[tip_name] = totals_by_tip.get(tip_name, 0) + int(moves)

        return {"per_step": per_step, "totals_by_tip": totals_by_tip}

    def compute_min_initials_with_tips(
        self,
        use_initial: bool = False,
        external_only: bool = True,
    ) -> Dict[str, Any]:
        needs = self.compute_min_initials(use_initial=use_initial, external_only=external_only)
        step_tips: List[Dict[str, Any]] = []
        totals_by_tip: Dict[str, int] = {}

        for i, s in enumerate(self.steps):
            if s.get("action") != "transfer_liquid":
                continue
            ns = s["sources"].unit_count()
            nt = s["targets"].unit_count()
            moves = self._fanout_multiplier(ns, nt) or 0
            tip_name = s.get("tip") or self.recommend_tip(s["unit_volume"])  # step-specific tip
            totals_by_tip[self.get_tip_capacity(tip_name)] = totals_by_tip.get(tip_name, 0) + int(moves)

            step_tips.append({
                "idx": i,
                "tip": tip_name,
                "tip_capacity": self.get_tip_capacity(tip_name),
                "unit_volume": s["unit_volume"],
                "tips_used": moves,
            })
        return {"liquid_setup": needs, "step_tips": step_tips, "totals_by_tip": totals_by_tip}

    # ---------- unknown labels ----------
    def _index_to_letters(self, idx: int) -> str:
        """0->A, 1->B, ... 25->Z, 26->AA, 27->AB ... (Excel-like)"""
        s: List[str] = []
        idx = int(idx)
        while True:
            idx, r = divmod(idx, 26)
            s.append(chr(ord('A') + r))
            if idx == 0:
                break
            idx -= 1  # Excel-style carry
        return "".join(reversed(s))

    def _label_for_unknown(self, res: MaterialResource) -> str:
        """Assign a stable unknown-liquid label (A/B/C/...) per resource."""
        if res not in self._unknown_labels:
            lab = self._index_to_letters(self._unknown_label_counter)
            self._unknown_label_counter += 1
            self._unknown_labels[res] = lab
        return self._unknown_labels[res]


# 在这一步传输目前有的物料
class LabResource:
    def __init__(self):
        self.tipracks = []
        self.plates = []
        self.trash = []

    def add_tipracks(self, tiprack: List[TipRack]):
        self.tipracks.extend(tiprack)
    def add_plates(self, plate: List[Plate]):
        self.plates.extend(plate)
    def add_trash(self, trash: List[Plate]):
        self.trash.extend(trash)
    def get_resources_info(self) -> Dict[str, Any]:
        tipracks = [{"name": tr.name, "max_volume": tr.children[0].tracker._tip.maximal_volume, "count": len(tr.children)} for tr in self.tipracks]
        plates = [{"name": pl.name, "max_volume": pl.children[0].max_volume, "count": len(pl.children)} for pl in self.plates]
        trash = [{"name": t.name, "max_volume": t.children[0].max_volume, "count": len(t.children)} for t in self.trash]
        return {
            "tipracks": tipracks,
            "plates": plates,
            "trash": trash
        }

from typing import Dict, Any

class DefaultLayout:
    
    def __init__(self, product_name: str = "PRCXI9300"):
        self.labresource = None
        if product_name not in ["PRCXI9300", "PRCXI9320"]:
            raise ValueError(f"Unsupported product_name: {product_name}. Only 'PRCXI9300' and 'PRCXI9320' are supported.")

        if product_name == "PRCXI9300":
            self.rows = 2
            self.columns = 3
            self.layout = [1, 2, 3, 4, 5, 6]
            self.trash_slot = 3
            self.waste_liquid_slot = 6
        elif product_name == "PRCXI9320":
            self.rows = 3
            self.columns = 4
            self.layout = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
            self.trash_slot = 3
            self.waste_liquid_slot = 12

    def get_layout(self) -> Dict[str, Any]:
        return {
            "rows": self.rows,
            "columns": self.columns,
            "layout": self.layout,
            "trash_slot": self.trash_slot,
            "waste_liquid_slot": self.waste_liquid_slot
        }

    def get_trash_slot(self) -> int:
        return self.trash_slot
    
    def get_waste_liquid_slot(self) -> int:
        return self.waste_liquid_slot
    
    def set_liquid_handler_layout(self, product_name: str):
        if product_name == "PRCXI9300":
            self.rows = 2
            self.columns = 3
            self.layout = [1, 2, 3, 4, 5, 6]
            self.trash_slot = 3
            self.waste_liquid_slot = 6

        elif product_name == "PRCXI9320":
            self.rows = 3
            self.columns = 4
            self.layout = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
            self.trash_slot = 3
            self.waste_liquid_slot = 12

    def set_trash_slot(self, slot: int):
        self.trash_slot = slot

    def set_waste_liquid_slot(self, slot: int):
        self.waste_liquid_slot = slot

    def add_lab_resource(self, lab_resource: LabResource):
        self.labresource = lab_resource.get_resources_info()

    def recommend_layout(self, needs: Dict[str, Any]) -> Dict[str, Any]:
        """根据 needs 推荐布局"""
        liquid_info = needs['liquid_setup']
        tip_info = needs['totals_by_tip']  # 修改这里：直接访问 totals_by_tip
        print("当前实验所需物料信息：", liquid_info)
        print("当前实验所需枪头信息：", tip_info)
        print(self.labresource)

        for liquid in liquid_info:
            # total_volume = liquid.values()
            print(liquid)
            #print(f"资源 {liquid} 需要的总体积: {total_volume}")

if __name__ == "__main__":
    # ---- 资源：SUP 供液（X），中间板 R1（4 孔空），目标板 R2（4 孔空）----
    sup = MaterialResource("SUP", slot=5, well=[1], liquid_id="X", volume=10000)      
    r1  = MaterialResource("R1",  slot=6, well=[1,2,3,4,5,6,7,8])
    r2  = MaterialResource("R2",  slot=7, well=[1,2,3,4,5,6,7,8])

    pm = ProtocolManager()
    # 步骤1：SUP -> R1，1->N 扇出，每孔 50 uL（总 200 uL）
    pm.add_transfer(sup, r1, unit_volume=10.0)
    # 步骤2：R1 -> R2，N->N 对应，每对 25 uL（总 100 uL；来自 R1 中已存在的混合物 X）
    pm.add_transfer(r1, r2, unit_volume=120.0)

    out = pm.compute_min_initials_with_tips()
    



    # layout_planer = DefaultLayout('PRCXI9320')
    # print(layout_planer.get_layout())
    # print("回推最小需求：", out["liquid_setup"])     # {'SUP': {'X': 200.0}}
    # print("步骤枪头建议：", out["step_tips"]) # [{'idx':0,'tip':'TIP_200uL','unit_volume':50.0}, {'idx':1,'tip':'TIP_50uL','unit_volume':25.0}]

#     # 实际执行（可选）
#     transfer_liquid(sup, r1, unit_volume=50.0)
#     transfer_liquid(r1, r2, unit_volume=25.0)
#     print("执行后 SUP：", sup.get_resource())  # 总体积 -200
#     print("执行后 R1：",  r1.get_resource())   # 每孔 25 uL（50 进 -25 出）
#     print("执行后 R2：",  r2.get_resource())   # 每孔 25 uL


    from pylabrobot.resources.opentrons.tube_racks import *
    from pylabrobot.resources.opentrons.plates import *
    from pylabrobot.resources.opentrons.tip_racks import *
    from pylabrobot.resources.opentrons.reservoirs import *

    plate = [locals()['nest_96_wellplate_2ml_deep'](name="thermoscientificnunc_96_wellplate_2000ul"), locals()['corning_96_wellplate_360ul_flat'](name="corning_96_wellplate_360ul_flat")]
    tiprack = [locals()['opentrons_96_tiprack_300ul'](name="opentrons_96_tiprack_300ul"), locals()['opentrons_96_tiprack_1000ul'](name="opentrons_96_tiprack_1000ul")]
    trash = [locals()['axygen_1_reservoir_90ml'](name="axygen_1_reservoir_90ml")]

    from pprint import pprint

    lab_resource = LabResource()
    lab_resource.add_tipracks(tiprack)
    lab_resource.add_plates(plate)
    lab_resource.add_trash(trash)

    layout_planer = DefaultLayout('PRCXI9300')
    layout_planer.add_lab_resource(lab_resource)
    layout_planer.recommend_layout(out)