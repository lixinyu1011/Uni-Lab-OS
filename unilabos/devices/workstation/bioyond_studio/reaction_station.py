import json

from unilabos.devices.workstation.bioyond_studio.station import BioyondWorkstation
from unilabos.devices.workstation.bioyond_studio.config import (
    API_CONFIG, WORKFLOW_MAPPINGS, WORKFLOW_STEP_IDS, MATERIAL_TYPE_MAPPINGS,
    STATION_TYPES, DEFAULT_STATION_CONFIG
)


class BioyondReactionStation(BioyondWorkstation):
    def __init__(self, config: dict = None):
        super().__init__(config)

    # 工作流方法
    def reactor_taken_out(self):
        """反应器取出"""
        self.hardware_interface.append_to_workflow_sequence('{"web_workflow_name": "reactor_taken_out"}')
        reactor_taken_out_params = {"param_values": {}}
        self.hardware_interface.pending_task_params.append(reactor_taken_out_params)
        print(f"成功添加反应器取出工作流")
        print(f"当前队列长度: {len(self.hardware_interface.pending_task_params)}")
        return json.dumps({"suc": True})

    def reactor_taken_in(self, assign_material_name: str, cutoff: str = "900000", temperature: float = -10.00):
        """反应器放入"""
        self.append_to_workflow_sequence('{"web_workflow_name": "reactor_taken_in"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        step_id = WORKFLOW_STEP_IDS["reactor_taken_in"]["config"]
        reactor_taken_in_params = {
            "param_values": {
                step_id: [
                    {"m": 0, "n": 3, "Key": "cutoff", "Value": cutoff},
                    {"m": 0, "n": 3, "Key": "temperature", "Value": f"{temperature:.2f}"},
                    {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id}
                ]
            }
        }

        self.pending_task_params.append(reactor_taken_in_params)
        print(f"成功添加反应器放入参数: material={assign_material_name}->ID:{material_id}, cutoff={cutoff}, temp={temperature:.2f}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def solid_feeding_vials(self, material_id: str, time: str = "0", torque_variation: str = "1",
                            assign_material_name: str = None, temperature: float = 25.00):
        """固体进料小瓶"""
        self.append_to_workflow_sequence('{"web_workflow_name": "Solid_feeding_vials"}')
        material_id_m = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        feeding_id = WORKFLOW_STEP_IDS["solid_feeding_vials"]["feeding"]
        observe_id = WORKFLOW_STEP_IDS["solid_feeding_vials"]["observe"]

        solid_feeding_vials_params = {
            "param_values": {
                feeding_id: [
                    {"m": 0, "n": 3, "Key": "materialId", "Value": material_id},
                    {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id_m}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": torque_variation},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(solid_feeding_vials_params)
        print(f"成功添加固体进料小瓶参数: material_id={material_id}, time={time}min, temp={temperature:.2f}°C")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_vials_non_titration(self, volumeFormula: str, assign_material_name: str,
                                          titration_type: str = "1", time: str = "0",
                                          torque_variation: str = "1", temperature: float = 25.00):
        """液体进料小瓶(非滴定)"""
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding_vials(non-titration)"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_id = WORKFLOW_STEP_IDS["liquid_feeding_vials_non_titration"]["liquid"]
        observe_id = WORKFLOW_STEP_IDS["liquid_feeding_vials_non_titration"]["observe"]

        params = {
            "param_values": {
                liquid_id: [
                    {"m": 0, "n": 3, "Key": "volumeFormula", "Value": volumeFormula},
                    {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id},
                    {"m": 0, "n": 3, "Key": "titrationType", "Value": titration_type}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": torque_variation},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料小瓶(非滴定)参数: volume={volumeFormula}μL, material={assign_material_name}->ID:{material_id}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_solvents(self, assign_material_name: str, volume: str, titration_type: str = "1",
                               time: str = "360", torque_variation: str = "2", temperature: float = 25.00):
        """液体进料溶剂"""
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding_solvents"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_id = WORKFLOW_STEP_IDS["liquid_feeding_solvents"]["liquid"]
        observe_id = WORKFLOW_STEP_IDS["liquid_feeding_solvents"]["observe"]

        params = {
            "param_values": {
                liquid_id: [
                    {"m": 0, "n": 1, "Key": "titrationType", "Value": titration_type},
                    {"m": 0, "n": 1, "Key": "volume", "Value": volume},
                    {"m": 0, "n": 1, "Key": "assignMaterialName", "Value": material_id}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": torque_variation},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料溶剂参数: material={assign_material_name}->ID:{material_id}, volume={volume}μL")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_titration(self, volume_formula: str, assign_material_name: str, titration_type: str = "1",
                                time: str = "90", torque_variation: int = 2, temperature: float = 25.00):
        """液体进料(滴定)"""
        self.append_to_workflow_sequence('{"web_workflow_name": "Liquid_feeding(titration)"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_id = WORKFLOW_STEP_IDS["liquid_feeding_titration"]["liquid"]
        observe_id = WORKFLOW_STEP_IDS["liquid_feeding_titration"]["observe"]

        params = {
            "param_values": {
                liquid_id: [
                    {"m": 0, "n": 3, "Key": "volumeFormula", "Value": volume_formula},
                    {"m": 0, "n": 3, "Key": "titrationType", "Value": titration_type},
                    {"m": 0, "n": 3, "Key": "assignMaterialName", "Value": material_id}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": str(torque_variation)},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料滴定参数: volume={volume_formula}μL, material={assign_material_name}->ID:{material_id}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})

    def liquid_feeding_beaker(self, volume: str = "35000", assign_material_name: str = "BAPP",
                             time: str = "0", torque_variation: str = "1", titrationType: str = "1",
                             temperature: float = 25.00):
        """液体进料烧杯"""
        self.append_to_workflow_sequence('{"web_workflow_name": "liquid_feeding_beaker"}')
        material_id = self._get_material_id_by_name(assign_material_name)

        if isinstance(temperature, str):
            temperature = float(temperature)

        liquid_id = WORKFLOW_STEP_IDS["liquid_feeding_beaker"]["liquid"]
        observe_id = WORKFLOW_STEP_IDS["liquid_feeding_beaker"]["observe"]

        params = {
            "param_values": {
                liquid_id: [
                    {"m": 0, "n": 2, "Key": "volume", "Value": volume},
                    {"m": 0, "n": 2, "Key": "assignMaterialName", "Value": material_id},
                    {"m": 0, "n": 2, "Key": "titrationType", "Value": titrationType}
                ],
                observe_id: [
                    {"m": 1, "n": 0, "Key": "time", "Value": time},
                    {"m": 1, "n": 0, "Key": "torqueVariation", "Value": torque_variation},
                    {"m": 1, "n": 0, "Key": "temperature", "Value": f"{temperature:.2f}"}
                ]
            }
        }

        self.pending_task_params.append(params)
        print(f"成功添加液体进料烧杯参数: volume={volume}μL, material={assign_material_name}->ID:{material_id}")
        print(f"当前队列长度: {len(self.pending_task_params)}")
        return json.dumps({"suc": True})