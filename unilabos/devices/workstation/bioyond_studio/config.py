# config.py
"""
配置文件 - 包含所有配置信息和映射关系
"""

# API配置
API_CONFIG = {
    "api_key": "",
    "api_host": ""
}

# 站点类型配置
STATION_TYPES = {
    "REACTION": "reaction_station",      # 仅反应站
    "DISPENSING": "dispensing_station",  # 仅配液站
    "HYBRID": "hybrid_station"           # 混合模式
}

# 默认站点配置
DEFAULT_STATION_CONFIG = {
    "station_type": STATION_TYPES["REACTION"],  # 默认反应站模式
    "enable_reaction_station": True,            # 是否启用反应站功能
    "enable_dispensing_station": False,         # 是否启用配液站功能
    "station_name": "BioyondReactionStation",   # 站点名称
    "description": "Bioyond反应工作站"          # 站点描述
}

# 工作流映射配置
WORKFLOW_MAPPINGS = {
    "reactor_taken_out": "",
    "reactor_taken_in": "",
    "Solid_feeding_vials": "",
    "Liquid_feeding_vials(non-titration)": "",
    "Liquid_feeding_solvents": "",
    "Liquid_feeding(titration)": "",
    "liquid_feeding_beaker": "",
    "Drip_back": "",
}

# 工作流名称到DisplaySectionName的映射
WORKFLOW_TO_SECTION_MAP = {
    'reactor_taken_in': '反应器放入',
    'liquid_feeding_beaker': '液体投料-烧杯',
    'Liquid_feeding_vials(non-titration)': '液体投料-小瓶（非滴定）',
    'Liquid_feeding_solvents': '液体投料-溶剂',
    'Solid_feeding_vials': '固体投料-小瓶',
    'Liquid_feeding(titration)': '液体投料-滴定',
    'reactor_taken_out': '反应器取出'
}

# 库位映射配置
LOCATION_MAPPING = {
    'A01': '',
    'A02': '',
    'A03': '',
    'A04': '',
    'A05': '',
    'A06': '',
    'A07': '',
    'A08': '',
    'B01': '',
    'B02': '',
    'B03': '',
    'B04': '',
    'B05': '',
    'B06': '',
    'B07': '',
    'B08': '',
    'C01': '',
    'C02': '',
    'C03': '',
    'C04': '',
    'C05': '',
    'C06': '',
    'C07': '',
    'C08': '',
    'D01': '',
    'D02': '',
    'D03': '',
    'D04': '',
    'D05': '',
    'D06': '',
    'D07': '',
    'D08': '',
}

# 物料类型配置
MATERIAL_TYPE_IDS = {
    "样品板": "",
    "样品": "",
    "烧杯": ""
}

MATERIAL_TYPE_MAPPINGS = {
    "烧杯": "BIOYOND_PolymerStation_1FlaskCarrier",
    "试剂瓶": "BIOYOND_PolymerStation_1BottleCarrier",
    "样品板": "BIOYOND_PolymerStation_6VialCarrier",
}

# 步骤参数配置（各工作流的步骤UUID）
WORKFLOW_STEP_IDS = {
    "reactor_taken_in": {
        "config": ""
    },
    "liquid_feeding_beaker": {
        "liquid": "",
        "observe": ""
    },
    "liquid_feeding_vials_non_titration": {
        "liquid": "",
        "observe": ""
    },
    "liquid_feeding_solvents": {
        "liquid": "",
        "observe": ""
    },
    "solid_feeding_vials": {
        "feeding": "",
        "observe": ""
    },
    "liquid_feeding_titration": {
        "liquid": "",
        "observe": ""
    },
    "drip_back": {
        "liquid": "",
        "observe": ""
    }
}
