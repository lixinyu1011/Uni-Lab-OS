def get_vessel(vessel):
    """
    统一处理vessel参数，返回vessel_id和vessel_data。

    Args:
        vessel: 可以是一个字典或字符串，表示vessel的ID或数据。

    Returns:
        tuple: 包含vessel_id和vessel_data。
    """
    if isinstance(vessel, dict):
        if "id" not in vessel:
            vessel_id = list(vessel.values())[0].get("id", "")
        else:
            vessel_id = vessel.get("id", "")
        vessel_data = vessel.get("data", {})
    else:
        vessel_id = str(vessel)
        vessel_data = {}
    return vessel_id, vessel_data
