
import json
from typing import Sequence, Optional, List, Union, Literal
json_path = "/Users/guangxinzhang/Documents/Deep Potential/opentrons/convert/protocols/enriched_steps/sci-lucif-assay4.json"

with open(json_path, "r") as f:
    data = json.load(f)

transfer_example = data[0]
#print(transfer_example)


temp_protocol = []

sources = transfer_example["sources"]  # Assuming sources is a list of Container objects
targets = transfer_example["targets"]  # Assuming targets is a list of Container objects
tip_racks = transfer_example["tip_racks"]  # Assuming tip_racks is a list of TipRack objects
asp_vols = transfer_example["asp_vols"]  # Assuming asp_vols is a list of volumes

def transfer_liquid(
        #self,
        sources,#: Sequence[Container],
        targets,#: Sequence[Container],
        tip_racks,#: Sequence[TipRack],
        # *,
        # use_channels: Optional[List[int]] = None,
        asp_vols: Union[List[float], float],
        # dis_vols: Union[List[float], float],
        # asp_flow_rates: Optional[List[Optional[float]]] = None,
        # dis_flow_rates: Optional[List[Optional[float]]] = None,
        # offsets,#: Optional[List[]] = None,
        # touch_tip: bool = False,
        # liquid_height: Optional[List[Optional[float]]] = None,
        # blow_out_air_volume: Optional[List[Optional[float]]] = None,
        # spread: Literal["wide", "tight", "custom"] = "wide",
        # is_96_well: bool = False,
        # mix_stage: Optional[Literal["none", "before", "after", "both"]] = "none",
        # mix_times,#: Optional[list() = None,
        # mix_vol: Optional[int] = None,
        # mix_rate: Optional[int] = None,
        # mix_liquid_height: Optional[float] = None,
        # delays: Optional[List[int]] = None,
        # none_keys: List[str] = []
    ):
        # -------- Build Biomek transfer step --------
        # 1) Construct default parameter scaffold (values mirror Biomek “Transfer” block).
        transfer_params = {
            "Span8": False,
            "Pod": "Pod1",
            "items": {},                       # to be filled below
            "Wash": False,
            "Dynamic?": True,
            "AutoSelectActiveWashTechnique": False,
            "ActiveWashTechnique": "",
            "ChangeTipsBetweenDests": True,
            "ChangeTipsBetweenSources": False,
            "DefaultCaption": "",              # filled after we know first pair/vol
            "UseExpression": False,
            "LeaveTipsOn": False,
            "MandrelExpression": "",
            "Repeats": "1",
            "RepeatsByVolume": False,
            "Replicates": "1",
            "ShowTipHandlingDetails": False,
            "ShowTransferDetails": True,
            "Solvent": "Water",
            "Span8Wash": False,
            "Span8WashVolume": "2",
            "Span8WasteVolume": "1",
            "SplitVolume": False,
            "SplitVolumeCleaning": False,
            "Stop": "Destinations",
            "TipLocation": "BC1025F",
            "UseCurrentTips": False,
            "UseDisposableTips": False,
            "UseFixedTips": False,
            "UseJIT": True,
            "UseMandrelSelection": True,
            "UseProbes": [True, True, True, True, True, True, True, True],
            "WashCycles": "3",
            "WashVolume": "110%",
            "Wizard": False
        }

        # 2) Build the items mapping (source/dest/volume triplets).
        #    Priority: user‑provided sources, targets, dis_vols.
        items: dict = {}
        for idx, (src, dst) in enumerate(zip(sources, targets)):
            items[str(idx)] = {
                "Source": str(src),
                "Destination": str(dst),
                "Volume": asp_vols[idx] 
            }
        transfer_params["items"] = items

        return transfer_params

        # # 3) Set a readable caption using the first source/target if available.
        # if items:
        #     first_item = next(iter(items.values()))
        #     transfer_params["DefaultCaption"] = (
        #         f"Transfer {first_item['Volume']} µL from "
        #         f"{first_item['Source']} to {first_item['Destination']}"
        #     )

        # # 4) Append the fully‑formed step to the temp protocol.
        # self.temp_protocol.setdefault("steps", []).append({"transfer": transfer_params})



action = transfer_liquid(sources=sources,targets=targets,tip_racks=tip_racks)
print(action)
# print(action)




"""
      "transfer": {

        "items": {},
        "Wash": false,
        "Dynamic?": true,
        "AutoSelectActiveWashTechnique": false,
        "ActiveWashTechnique": "",
        "ChangeTipsBetweenDests": true,
        "ChangeTipsBetweenSources": false,
        "DefaultCaption": "Transfer 100 µL from P13 to P3",
        "UseExpression": false,
        "LeaveTipsOn": false,
        "MandrelExpression": "",
        "Repeats": "1",
        "RepeatsByVolume": false,
        "Replicates": "1",
        "ShowTipHandlingDetails": false,
        "ShowTransferDetails": true,
        "Solvent": "Water",
        "Span8Wash": false,
        "Span8WashVolume": "2",
        "Span8WasteVolume": "1",
        "SplitVolume": false,
        "SplitVolumeCleaning": false,
        "Stop": "Destinations",
        "TipLocation": "BC1025F",
        "UseCurrentTips": false,
        "UseDisposableTips": false,
        "UseFixedTips": false,
        "UseJIT": true,
        "UseMandrelSelection": true,
        "UseProbes": [true, true, true, true, true, true, true, true],
        "WashCycles": "3",
        "WashVolume": "110%",
        "Wizard": false
"""




