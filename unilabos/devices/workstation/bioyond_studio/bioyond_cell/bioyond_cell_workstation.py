# -*- coding: utf-8 -*-
from cgi import print_arguments
from doctest import debug
from typing import Dict, Any, List, Optional
import requests
from pathlib import Path
import pandas as pd
import time
from datetime import datetime, timedelta
import re
import threading
import os
import socket

from urllib3 import response
from unilabos.devices.workstation.workstation_base import WorkstationBase
from unilabos.devices.workstation.bioyond_studio.station import BioyondWorkstation
from unilabos.devices.workstation.bioyond_studio.config import (
    BIOYOND_FULL_CONFIG, WORKFLOW_MAPPINGS, MATERIAL_TYPE_MAPPINGS, WAREHOUSE_MAPPING
)
from unilabos.devices.workstation.workstation_http_service import WorkstationHTTPService
from unilabos.utils.log import logger

def _iso_local_now_ms() -> str:
    # 文档要求：到毫秒 + Z，例如 2025-08-15T05:43:22.814Z
    dt = datetime.now()
    # print(dt)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + f"{int(dt.microsecond/1000):03d}Z"


class BioyondCellWorkstation(BioyondWorkstation):
    """
    集成 Bioyond LIMS 的工作站示例，
    覆盖：入库(2.17/2.18) → 新建实验(2.14) → 启动调度(2.7) →
    运行中推送：物料变更(2.24)、步骤完成(2.21)、订单完成(2.23) →
    查询实验(2.5/2.6) → 3-2-1 转运(2.32) → 样品/废料取出(2.28)
    """

    def __init__(
        self,
        bioyond_config: Optional[Dict[str, Any]] = None,
        station_resource: Optional[Dict[str, Any]] = None,
        *args, **kwargs,
        ):

        # 使用统一配置，支持自定义覆盖
        self.bioyond_config = bioyond_config or {
            **BIOYOND_FULL_CONFIG,  # 从 config.py 加载完整配置
            "workflow_mappings": WORKFLOW_MAPPINGS,
            "material_type_mappings": MATERIAL_TYPE_MAPPINGS,
            "warehouse_mapping": WAREHOUSE_MAPPING
        }
        self.debug_mode = self.bioyond_config["debug_mode"]
        self.http_service_started = False
        deck = kwargs.pop("deck", None)
        self.device_id = kwargs.pop("device_id", "bioyond_cell_workstation")
        super().__init__(bioyond_config=self.bioyond_config, deck=deck, station_resource=station_resource, *args, **kwargs)
        # 步骤通量任务通知铃
        self._pending_events: dict[str, threading.Event] = {}
        logger.info(f"Bioyond工作站初始化完成 (debug_mode={self.debug_mode})")

        # 实例化并在后台线程启动 HTTP 报送服务
        self.order_status = {} # 记录任务完成情况，用于接受bioyond post信息和反馈信息，尤其用于硬件查询和物料信息变化
        try:
            logger.info("准备开始unilab_HTTP后台线程")
            t = threading.Thread(target=self._start_http_service, daemon=True, name="unilab_http")
            t.start()
        except Exception as e:
            logger.error(f"unilab-HTTP后台线程启动失败: {e}")

    # http报送服务
    def process_step_finish_report(self, report_request):
        stepId = report_request.data.get("stepId")
        logger.info(f"步骤完成: stepId: {stepId}, stepName:{report_request.data.get('stepName')}")
        return report_request.data.get('executionStatus')

    def process_sample_finish_report(self, report_request):
        logger.info(f"通量完成: {report_request.data.get('sampleId')}")
        return {"status": "received"}

    def process_order_finish_report(self, report_request, used_materials=None):
        order_code = report_request.data.get("orderCode")
        status = report_request.data.get("status")
        logger.info(f"report_request: {report_request}")

        logger.info(f"任务完成: {order_code}, status={status}")
        
        # 记录订单状态码
        if order_code:
            self.order_status[order_code] = status
        
        self._set_pending_event(order_code)
        return {"status": "received"}

    def _set_pending_event(self, taskname: Optional[str]) -> None:
        if not taskname:
            return
        event = self._pending_events.get(taskname)
        if event is None:
            event = threading.Event()
            self._pending_events[taskname] = event
        event.set()

    def _wait_for_order_completion(self, order_code: Optional[str], timeout: int = 600) -> bool:
        if not order_code:
            logger.warning("无法等待任务完成：order_code 为空")
            return False
        event = self._pending_events.get(order_code)
        if event is None:
            event = threading.Event()
            self._pending_events[order_code] = event
        elif event.is_set():
            logger.info(f"任务 {order_code} 在等待之前已完成")
            self._pending_events.pop(order_code, None)
            return True
        logger.info(f"等待任务 {order_code} 完成 (timeout={timeout}s)")
        finished = event.wait(timeout)
        if not finished:
            logger.warning(f"等待任务 {order_code} 完成超时（{timeout}s）")
        self._pending_events.pop(order_code, None)
        return finished

    def _wait_for_response_orders(self, response: Dict[str, Any], context: str, timeout: int = 600) -> None:
        order_codes = self._extract_order_codes(response)
        if not order_codes:
            logger.warning(f"{context} 响应中未找到 orderCode，无法跟踪任务完成")
            return
        for code in order_codes:
            finished = self._wait_for_order_completion(code, timeout=timeout)
            if finished:
                # 检查订单返回码是否为30（正常完成）
                status = self.order_status.get(code)
                if status == 30 or status == "30":
                    logger.info(f"订单 {code} 成功完成，状态码: {status}")
                else:
                    logger.warning(f"订单 {code} 完成但状态码异常: {status} (期望: 30, -11=异常停止, -12=人工停止)")
                # 清理状态记录
                self.order_status.pop(code, None)
            else:
                logger.error(f"订单 {code} 等待超时，未收到完成通知")

    @staticmethod
    def _extract_order_codes(response: Dict[str, Any]) -> List[str]:
        order_codes: List[str] = []
        if not isinstance(response, dict):
            return order_codes
        data = response.get("data")
        keys = ["orderCode", "order_code", "orderId", "order_id"]
        if isinstance(data, dict):
            for key in keys:
                if key in data and data[key]:
                    order_codes.append(str(data[key]))
            if not order_codes and "orders" in data and isinstance(data["orders"], list):
                for order in data["orders"]:
                    if isinstance(order, dict):
                        for key in keys:
                            if key in order and order[key]:
                                order_codes.append(str(order[key]))
        elif isinstance(data, list):
            for item in data:
                if isinstance(item, dict):
                    for key in keys:
                        if key in item and item[key]:
                            order_codes.append(str(item[key]))
        elif isinstance(data, str):
            if data:
                order_codes.append(data)
        meta = response.get("orderCode")
        if meta:
            order_codes.append(str(meta))
        # 去重
        seen = set()
        unique_codes: List[str] = []
        for code in order_codes:
            if code not in seen:
                seen.add(code)
                unique_codes.append(code)
        return unique_codes


    def _start_http_service(self, host: Optional[str] = None, port: Optional[int] = None) -> None:
        host = host or self.bioyond_config.get("HTTP_host", "")
        port = port or self.bioyond_config.get("HTTP_port", )

        logger.info("准备开始unilab_HTTP服务")
        try:
            self.service = WorkstationHTTPService(self, host=host, port=port)
            logger.info("WorkstationHTTPService 实例化完成")
            self.service.start()
            self.http_service_started = True
            logger.info(f"WorkstationHTTPService成功启动: {host}:{port}")
            
            # 启动成功后，上报本机推送地址（3.36）
            try:
                # 优先使用配置中的 report_ip
                report_ip = self.bioyond_config.get("report_ip", "").strip()
                
                # 如果配置中没有指定 report_ip，且监听地址是 0.0.0.0，则自动检测
                if not report_ip and host in ("0.0.0.0", ""):
                    # 从 Bioyond 配置中提取服务器地址
                    bioyond_server = self.bioyond_config.get("base_url", "")
                    if bioyond_server:
                        import urllib.parse
                        parsed = urllib.parse.urlparse(bioyond_server)
                    
                elif not report_ip:
                    # 如果没有配置 report_ip，使用监听地址
                    report_ip = host
                
                r = self.update_push_ip(report_ip, port)
                logger.info(f"向 Bioyond 报送推送地址: {report_ip}:{port}, 结果: {r}")
            except Exception as e:
                logger.warning(f"调用更新推送IP接口失败: {e}")
            
            #一直挂着，直到进程退出
            while True:
                time.sleep(1)
        except Exception as e:
            self.http_service_started = False # 调试用
            logger.error(f"启动WorkstationHTTPService失败: {e}", exc_info=True)

    # -------------------- 基础HTTP封装 --------------------
    def _url(self, path: str) -> str:

        return f"{self.bioyond_config['base_url'].rstrip('/')}/{path.lstrip('/')}"

    def _post_lims(self, path: str, data: Optional[Any] = None) -> Dict[str, Any]:
        """LIMS API：大多数接口用 {apiKey/requestTime,data} 包装"""
        payload = {
            "apiKey": self.bioyond_config["api_key"],
            "requestTime": _iso_local_now_ms()
        }
        if data is not None:
            payload["data"] = data

        if self.debug_mode:
            # 模拟返回，不发真实请求
            logger.info(f"[DEBUG] POST {path} with payload={payload}")
            return {"debug": True, "url": self._url(path), "payload": payload, "status": "ok"}

        try:
            response = requests.post(
                self._url(path), 
                json=payload,
                timeout=self.bioyond_config.get("timeout", 30),
                headers={"Content-Type": "application/json"}
            ) # 拼接网址+post bioyond接口
            response.raise_for_status()
            return response.json()
        except Exception as e:
            logger.info(f"{self.bioyond_config['base_url'].rstrip('/')}/{path.lstrip('/')}")
            logger.error(f"POST {path} 失败: {e}")
            return {"error": str(e)}

    def _put_lims(self, path: str, data: Optional[Any] = None) -> Dict[str, Any]:
        """LIMS API：PUT {apiKey/requestTime,data} 包装"""
        payload = {
            "apiKey": self.bioyond_config["api_key"],
            "requestTime": _iso_local_now_ms()
        }
        if data is not None:
            payload["data"] = data

        if self.debug_mode:
            logger.info(f"[DEBUG] PUT {path} with payload={payload}")
            return {"debug": True, "url": self._url(path), "payload": payload, "status": "ok"}

        try:
            response = requests.put(
                self._url(path),
                json=payload,
                timeout=self.bioyond_config.get("timeout", 30),
                headers={"Content-Type": "application/json"}
            )
            response.raise_for_status()
            return response.json()
        except Exception as e:
            logger.info(f"{self.bioyond_config['base_url'].rstrip('/')}/{path.lstrip('/')}")
            logger.error(f"PUT {path} 失败: {e}")
            return {"error": str(e)}

    # -------------------- 3.36 更新推送 IP 地址 --------------------
    def update_push_ip(self, ip: Optional[str] = None, port: Optional[int] = None) -> Dict[str, Any]:
        """
        3.36 更新推送 IP 地址接口（PUT）
        URL: /api/lims/order/ip-config
        请求体：{ apiKey, requestTime, data: { ip, port } }
        """
        target_ip = ip or self.bioyond_config.get("HTTP_host", "")
        target_port = int(port or self.bioyond_config.get("HTTP_port", 0))
        data = {"ip": target_ip, "port": target_port}

        # 固定接口路径，不做其他路径兼容
        path = "/api/lims/order/ip-config"
        return self._put_lims(path, data)

    # -------------------- 单点接口封装 --------------------
    # 2.17 入库物料（单个）
    def storage_inbound(self, material_id: str, location_id: str) -> Dict[str, Any]:
        return self._post_lims("/api/lims/storage/inbound", {
            "materialId": material_id,
            "locationId": location_id
        })

    # 2.18 批量入库（多个）
    def storage_batch_inbound(self, items: List[Dict[str, str]]) -> Dict[str, Any]:
        """
        items = [{"materialId": "...", "locationId": "..."}, ...]
        """
        return self._post_lims("/api/lims/storage/batch-inbound", items)



    def auto_feeding4to3(
        self,
        # ★ 修改点：默认模板路径
        xlsx_path: Optional[str] = "/Users/calvincao/Desktop/work/uni-lab-all/Uni-Lab-OS/unilabos/devices/workstation/bioyond_studio/bioyond_cell/样品导入模板.xlsx",
        # ---------------- WH4 - 加样头面 (Z=1, 12个点位) ----------------
        WH4_x1_y1_z1_1_materialName: str = "", WH4_x1_y1_z1_1_quantity: float = 0.0,
        WH4_x2_y1_z1_2_materialName: str = "", WH4_x2_y1_z1_2_quantity: float = 0.0,
        WH4_x3_y1_z1_3_materialName: str = "", WH4_x3_y1_z1_3_quantity: float = 0.0,
        WH4_x4_y1_z1_4_materialName: str = "", WH4_x4_y1_z1_4_quantity: float = 0.0,
        WH4_x5_y1_z1_5_materialName: str = "", WH4_x5_y1_z1_5_quantity: float = 0.0,
        WH4_x1_y2_z1_6_materialName: str = "", WH4_x1_y2_z1_6_quantity: float = 0.0,
        WH4_x2_y2_z1_7_materialName: str = "", WH4_x2_y2_z1_7_quantity: float = 0.0,
        WH4_x3_y2_z1_8_materialName: str = "", WH4_x3_y2_z1_8_quantity: float = 0.0,
        WH4_x4_y2_z1_9_materialName: str = "", WH4_x4_y2_z1_9_quantity: float = 0.0,
        WH4_x5_y2_z1_10_materialName: str = "", WH4_x5_y2_z1_10_quantity: float = 0.0,
        WH4_x1_y3_z1_11_materialName: str = "", WH4_x1_y3_z1_11_quantity: float = 0.0,
        WH4_x2_y3_z1_12_materialName: str = "", WH4_x2_y3_z1_12_quantity: float = 0.0,

        # ---------------- WH4 - 原液瓶面 (Z=2, 9个点位) ----------------
        WH4_x1_y1_z2_1_materialName: str = "", WH4_x1_y1_z2_1_quantity: float = 0.0, WH4_x1_y1_z2_1_materialType: str = "", WH4_x1_y1_z2_1_targetWH: str = "",
        WH4_x2_y1_z2_2_materialName: str = "", WH4_x2_y1_z2_2_quantity: float = 0.0, WH4_x2_y1_z2_2_materialType: str = "", WH4_x2_y1_z2_2_targetWH: str = "",
        WH4_x3_y1_z2_3_materialName: str = "", WH4_x3_y1_z2_3_quantity: float = 0.0, WH4_x3_y1_z2_3_materialType: str = "", WH4_x3_y1_z2_3_targetWH: str = "",
        WH4_x1_y2_z2_4_materialName: str = "", WH4_x1_y2_z2_4_quantity: float = 0.0, WH4_x1_y2_z2_4_materialType: str = "", WH4_x1_y2_z2_4_targetWH: str = "",
        WH4_x2_y2_z2_5_materialName: str = "", WH4_x2_y2_z2_5_quantity: float = 0.0, WH4_x2_y2_z2_5_materialType: str = "", WH4_x2_y2_z2_5_targetWH: str = "",
        WH4_x3_y2_z2_6_materialName: str = "", WH4_x3_y2_z2_6_quantity: float = 0.0, WH4_x3_y2_z2_6_materialType: str = "", WH4_x3_y2_z2_6_targetWH: str = "",
        WH4_x1_y3_z2_7_materialName: str = "", WH4_x1_y3_z2_7_quantity: float = 0.0, WH4_x1_y3_z2_7_materialType: str = "", WH4_x1_y3_z2_7_targetWH: str = "",
        WH4_x2_y3_z2_8_materialName: str = "", WH4_x2_y3_z2_8_quantity: float = 0.0, WH4_x2_y3_z2_8_materialType: str = "", WH4_x2_y3_z2_8_targetWH: str = "",
        WH4_x3_y3_z2_9_materialName: str = "", WH4_x3_y3_z2_9_quantity: float = 0.0, WH4_x3_y3_z2_9_materialType: str = "", WH4_x3_y3_z2_9_targetWH: str = "",

        # ---------------- WH3 - 人工堆栈 (Z=3, 15个点位) ----------------
        WH3_x1_y1_z3_1_materialType: str = "", WH3_x1_y1_z3_1_materialId: str = "", WH3_x1_y1_z3_1_quantity: float = 0,
        WH3_x2_y1_z3_2_materialType: str = "", WH3_x2_y1_z3_2_materialId: str = "", WH3_x2_y1_z3_2_quantity: float = 0,
        WH3_x3_y1_z3_3_materialType: str = "", WH3_x3_y1_z3_3_materialId: str = "", WH3_x3_y1_z3_3_quantity: float = 0,
        WH3_x1_y2_z3_4_materialType: str = "", WH3_x1_y2_z3_4_materialId: str = "", WH3_x1_y2_z3_4_quantity: float = 0,
        WH3_x2_y2_z3_5_materialType: str = "", WH3_x2_y2_z3_5_materialId: str = "", WH3_x2_y2_z3_5_quantity: float = 0,
        WH3_x3_y2_z3_6_materialType: str = "", WH3_x3_y2_z3_6_materialId: str = "", WH3_x3_y2_z3_6_quantity: float = 0,
        WH3_x1_y3_z3_7_materialType: str = "", WH3_x1_y3_z3_7_materialId: str = "", WH3_x1_y3_z3_7_quantity: float = 0,
        WH3_x2_y3_z3_8_materialType: str = "", WH3_x2_y3_z3_8_materialId: str = "", WH3_x2_y3_z3_8_quantity: float = 0,
        WH3_x3_y3_z3_9_materialType: str = "", WH3_x3_y3_z3_9_materialId: str = "", WH3_x3_y3_z3_9_quantity: float = 0,
        WH3_x1_y4_z3_10_materialType: str = "", WH3_x1_y4_z3_10_materialId: str = "", WH3_x1_y4_z3_10_quantity: float = 0,
        WH3_x2_y4_z3_11_materialType: str = "", WH3_x2_y4_z3_11_materialId: str = "", WH3_x2_y4_z3_11_quantity: float = 0,
        WH3_x3_y4_z3_12_materialType: str = "", WH3_x3_y4_z3_12_materialId: str = "", WH3_x3_y4_z3_12_quantity: float = 0,
        WH3_x1_y5_z3_13_materialType: str = "", WH3_x1_y5_z3_13_materialId: str = "", WH3_x1_y5_z3_13_quantity: float = 0,
        WH3_x2_y5_z3_14_materialType: str = "", WH3_x2_y5_z3_14_materialId: str = "", WH3_x2_y5_z3_14_quantity: float = 0,
        WH3_x3_y5_z3_15_materialType: str = "", WH3_x3_y5_z3_15_materialId: str = "", WH3_x3_y5_z3_15_quantity: float = 0,
    ):
        """
        自动化上料（支持两种模式）
        - Excel 路径存在 → 从 Excel 模板解析
        - Excel 路径不存在 → 使用手动参数
        """
        items: List[Dict[str, Any]] = []

        # ---------- 模式 1: Excel 导入 ----------
        if xlsx_path:
            path = Path(xlsx_path)
            if path.exists():   # ★ 修改点：路径存在才加载
                try:
                    df = pd.read_excel(path, sheet_name=0, header=None, engine="openpyxl")
                except Exception as e:
                    raise RuntimeError(f"读取 Excel 失败：{e}")

                # 四号手套箱加样头面
                for _, row in df.iloc[1:13, 2:7].iterrows():
                    if pd.notna(row[5]):
                        items.append({
                            "sourceWHName": "四号手套箱堆栈",
                            "posX": int(row[2]), "posY": int(row[3]), "posZ": int(row[4]),
                            "materialName": str(row[5]).strip(),
                            "quantity": float(row[6]) if pd.notna(row[6]) else 0.0,
                        })
                # 四号手套箱原液瓶面
                for _, row in df.iloc[14:23, 2:9].iterrows():
                    if pd.notna(row[5]):
                        items.append({
                            "sourceWHName": "四号手套箱堆栈",
                            "posX": int(row[2]), "posY": int(row[3]), "posZ": int(row[4]),
                            "materialName": str(row[5]).strip(),
                            "quantity": float(row[6]) if pd.notna(row[6]) else 0.0,
                            "materialType": str(row[7]).strip() if pd.notna(row[7]) else "",
                            "targetWH": str(row[8]).strip() if pd.notna(row[8]) else "",
                        })
                # 三号手套箱人工堆栈
                for _, row in df.iloc[25:40, 2:7].iterrows():
                    if pd.notna(row[5]) or pd.notna(row[6]):
                        items.append({
                            "sourceWHName": "三号手套箱人工堆栈",
                            "posX": int(row[2]), "posY": int(row[3]), "posZ": int(row[4]),
                            "materialType": str(row[5]).strip() if pd.notna(row[5]) else "",
                            "materialId": str(row[6]).strip() if pd.notna(row[6]) else "",
                            "quantity": 1
                        })
            else:
                logger.warning(f"未找到 Excel 文件 {xlsx_path}，自动切换到手动参数模式。")

        # ---------- 模式 2: 手动填写 ----------
        if not items:
            params = locals()
            for name, value in params.items():
                if name.startswith("四号手套箱堆栈") and "materialName" in name and value:
                    idx = name.split("_")
                    items.append({
                        "sourceWHName": "四号手套箱堆栈",
                        "posX": int(idx[1][1:]), "posY": int(idx[2][1:]), "posZ": int(idx[3][1:]),
                        "materialName": value,
                        "quantity": float(params.get(name.replace("materialName", "quantity"), 0.0))
                    })
                elif name.startswith("四号手套箱堆栈") and "materialType" in name and (value or params.get(name.replace("materialType", "materialName"), "")):
                    idx = name.split("_")
                    items.append({
                        "sourceWHName": "四号手套箱堆栈",
                        "posX": int(idx[1][1:]), "posY": int(idx[2][1:]), "posZ": int(idx[3][1:]),
                        "materialName": params.get(name.replace("materialType", "materialName"), ""),
                        "quantity": float(params.get(name.replace("materialType", "quantity"), 0.0)),
                        "materialType": value,
                        "targetWH": params.get(name.replace("materialType", "targetWH"), ""),
                    })
                elif name.startswith("三号手套箱人工堆栈") and "materialType" in name and (value or params.get(name.replace("materialType", "materialId"), "")):
                    idx = name.split("_")
                    items.append({
                        "sourceWHName": "三号手套箱人工堆栈",
                        "posX": int(idx[1][1:]), "posY": int(idx[2][1:]), "posZ": int(idx[3][1:]),
                        "materialType": value,
                        "materialId": params.get(name.replace("materialType", "materialId"), ""),
                        "quantity": int(params.get(name.replace("materialType", "quantity"), 1)),
                    })

        if not items:
            logger.warning("没有有效的上料条目，已跳过提交。")
            return {"code": 0, "message": "no valid items", "data": []}
        logger.info(items)
        response = self._post_lims("/api/lims/order/auto-feeding4to3", items)
        self._wait_for_response_orders(response, "auto_feeding4to3")
        return response




    # 3.30 自动化上料(老版本)
    def auto_feeding4to3_from_xlsx(self, xlsx_path: str) -> Dict[str, Any]:
        """
        根据固定模板解析 Excel：
        - 四号手套箱加样头面 (2-13行, 3-7列)
        - 四号手套箱原液瓶面 (15-23行, 3-9列)        
        - 三号手套箱人工堆栈 (26-40行, 3-7列)
        """
        path = Path(xlsx_path)
        if not path.exists():
            raise FileNotFoundError(f"未找到 Excel 文件：{path}")

        try:
            df = pd.read_excel(path, sheet_name=0, header=None, engine="openpyxl")
        except Exception as e:
            raise RuntimeError(f"读取 Excel 失败：{e}")

        items: List[Dict[str, Any]] = []

        # 四号手套箱 - 加样头面（2-13行, 3-7列）
        for _, row in df.iloc[1:13, 2:7].iterrows():
            item = {
                "sourceWHName": "四号手套箱堆栈",
                "posX": int(row[2]),
                "posY": int(row[3]),
                "posZ": int(row[4]),
                "materialName": str(row[5]).strip() if pd.notna(row[5]) else "",
                "quantity": float(row[6]) if pd.notna(row[6]) else 0.0,
            }
            if item["materialName"]:
                items.append(item)

        # 四号手套箱 - 原液瓶面（15-23行, 3-9列）
        for _, row in df.iloc[14:23, 2:9].iterrows():
            item = {
                "sourceWHName": "四号手套箱堆栈",
                "posX": int(row[2]),
                "posY": int(row[3]),
                "posZ": int(row[4]),
                "materialName": str(row[5]).strip() if pd.notna(row[5]) else "",
                "quantity": float(row[6]) if pd.notna(row[6]) else 0.0,
                "materialType": str(row[7]).strip() if pd.notna(row[7]) else "",
                "targetWH": str(row[8]).strip() if pd.notna(row[8]) else "",
            }
            if item["materialName"]:
                items.append(item)

        # 三号手套箱人工堆栈（26-40行, 3-7列）
        for _, row in df.iloc[25:40, 2:7].iterrows():
            item = {
                "sourceWHName": "三号手套箱人工堆栈",
                "posX": int(row[2]),
                "posY": int(row[3]),
                "posZ": int(row[4]),
                "materialType": str(row[5]).strip() if pd.notna(row[5]) else "",
                "materialId": str(row[6]).strip() if pd.notna(row[6]) else "",
                "quantity": 1  # 默认数量1
            }
            if item["materialId"] or item["materialType"]:
                items.append(item)

        response = self._post_lims("/api/lims/order/auto-feeding4to3", items)
        self._wait_for_response_orders(response, "auto_feeding4to3_from_xlsx")
        return response

    def auto_batch_outbound_from_xlsx(self, xlsx_path: str) -> Dict[str, Any]:
        """
        3.31 自动化下料（Excel -> JSON -> POST /api/lims/storage/auto-batch-out-bound）
        """
        path = Path(xlsx_path)
        if not path.exists():
            raise FileNotFoundError(f"未找到 Excel 文件：{path}")

        try:
            df = pd.read_excel(path, sheet_name=0, engine="openpyxl")
        except Exception as e:
            raise RuntimeError(f"读取 Excel 失败：{e}")

        def pick(names: List[str]) -> Optional[str]:
            for n in names:
                if n in df.columns:
                    return n
            return None

        c_loc = pick(["locationId", "库位ID", "库位Id", "库位id"])
        c_wh  = pick(["warehouseId", "仓库ID", "仓库Id", "仓库id"])
        c_qty = pick(["数量", "quantity"])
        c_x   = pick(["x", "X", "posX", "坐标X"])
        c_y   = pick(["y", "Y", "posY", "坐标Y"])
        c_z   = pick(["z", "Z", "posZ", "坐标Z"])

        required = [c_loc, c_wh, c_qty, c_x, c_y, c_z]
        if any(c is None for c in required):
            raise KeyError("Excel 缺少必要列：locationId/warehouseId/数量/x/y/z（支持多别名，至少要能匹配到）。")

        def as_int(v, d=0):
            try:
                if pd.isna(v): return d
                return int(v)
            except Exception:
                try:
                    return int(float(v))
                except Exception:
                    return d

        def as_float(v, d=0.0):
            try:
                if pd.isna(v): return d
                return float(v)
            except Exception:
                return d

        def as_str(v, d=""):
            if v is None or (isinstance(v, float) and pd.isna(v)): return d
            s = str(v).strip()
            return s if s else d

        items: List[Dict[str, Any]] = []
        for _, row in df.iterrows():
            items.append({
                "locationId": as_str(row[c_loc]),
                "warehouseId": as_str(row[c_wh]),
                "quantity": as_float(row[c_qty]),
                "x": as_int(row[c_x]),
                "y": as_int(row[c_y]),
                "z": as_int(row[c_z]),
            })

        response = self._post_lims("/api/lims/storage/auto-batch-out-bound", items)
        self._wait_for_response_orders(response, "auto_batch_outbound_from_xlsx")
        return response

    # 2.14 新建实验
    def create_orders(self, xlsx_path: str) -> Dict[str, Any]:
        """
        从 Excel 解析并创建实验（2.14）
        约定：
        - batchId = Excel 文件名（不含扩展名）
        - 物料列：所有以 "(g)" 结尾（不再读取“总质量(g)”列）
        - totalMass 自动计算为所有物料质量之和
        - createTime 缺失或为空时自动填充为当前日期（YYYY/M/D）
        """
        path = Path(xlsx_path)
        if not path.exists():
            raise FileNotFoundError(f"未找到 Excel 文件：{path}")

        try:
            df = pd.read_excel(path, sheet_name=0, engine="openpyxl")
        except Exception as e:
            raise RuntimeError(f"读取 Excel 失败：{e}")

        # 列名容错：返回可选列名，找不到则返回 None
        def _pick(col_names: List[str]) -> Optional[str]:
            for c in col_names:
                if c in df.columns:
                    return c
            return None

        col_order_name = _pick(["配方ID", "orderName", "订单编号"])
        col_create_time = _pick(["创建日期", "createTime"])
        col_bottle_type = _pick(["配液瓶类型", "bottleType"])
        col_mix_time = _pick(["混匀时间(s)", "mixTime"])
        col_load = _pick(["扣电组装分液体积", "loadSheddingInfo"])
        col_pouch = _pick(["软包组装分液体积", "pouchCellInfo"])
        col_cond = _pick(["电导测试分液体积", "conductivityInfo"])
        col_cond_cnt = _pick(["电导测试分液瓶数", "conductivityBottleCount"])

        # 物料列：所有以 (g) 结尾
        material_cols = [c for c in df.columns if isinstance(c, str) and c.endswith("(g)")]
        if not material_cols:
            raise KeyError("未发现任何以“(g)”结尾的物料列，请检查表头。")

        batch_id = path.stem

        def _to_ymd_slash(v) -> str:
            # 统一为 "YYYY/M/D"；为空或解析失败则用当前日期
            if v is None or (isinstance(v, float) and pd.isna(v)) or str(v).strip() == "":
                ts = datetime.now()
            else:
                try:
                    ts = pd.to_datetime(v)
                except Exception:
                    ts = datetime.now()
            return f"{ts.year}/{ts.month}/{ts.day}"

        def _as_int(val, default=0) -> int:
            try:
                if pd.isna(val):
                    return default
                return int(val)
            except Exception:
                return default

        def _as_str(val, default="") -> str:
            if val is None or (isinstance(val, float) and pd.isna(val)):
                return default
            s = str(val).strip()
            return s if s else default

        orders: List[Dict[str, Any]] = []

        for idx, row in df.iterrows():
            mats: List[Dict[str, Any]] = []
            total_mass = 0.0

            for mcol in material_cols:
                val = row.get(mcol, None)
                if val is None or (isinstance(val, float) and pd.isna(val)):
                    continue
                try:
                    mass = float(val)
                except Exception:
                    continue
                if mass > 0:
                    mats.append({"name": mcol.replace("(g)", ""), "mass": mass})
                    total_mass += mass

            order_data = {
                "batchId": batch_id,
                "orderName": _as_str(row[col_order_name], default=f"{batch_id}_order_{idx+1}") if col_order_name else f"{batch_id}_order_{idx+1}",
                "createTime": _to_ymd_slash(row[col_create_time]) if col_create_time else _to_ymd_slash(None),
                "bottleType": _as_str(row[col_bottle_type], default="配液小瓶") if col_bottle_type else "配液小瓶",
                "mixTime": _as_int(row[col_mix_time]) if col_mix_time else 0,
                "loadSheddingInfo": _as_int(row[col_load]) if col_load else 0,
                "pouchCellInfo": _as_int(row[col_pouch]) if col_pouch else 0,
                "conductivityInfo": _as_int(row[col_cond]) if col_cond else 0,
                "conductivityBottleCount": _as_int(row[col_cond_cnt]) if col_cond_cnt else 0,
                "materialInfos": mats,
                "totalMass": round(total_mass, 4)  # 自动汇总
            }
            orders.append(order_data)

        # print(orders)
        while True:
            time.sleep(5)
            response = self._post_lims("/api/lims/order/orders", orders)
            if response.get("data", []):
                break
            logger.info(f"等待配液实验创建完成")


       
        # self.order_status[response["data"]["orderCode"]] = "running"

        # while True:
        #     time.sleep(5)
        #     if self.order_status.get(response["data"]["orderCode"], None) == "finished":
        #         logger.info(f"配液实验已完成 ，即将执行 3-2-1 转运")
        #         break
        #     logger.info(f"等待配液实验完成")

        # self.transfer_3_to_2_to_1()
        # self.wait_for_transfer_task()
        # logger.info(f"3-2-1 转运完成，返回结果")
        # return r321
        self._wait_for_response_orders(response, "create_orders", timeout=1800)
        return response

    # 2.7 启动调度
    def scheduler_start(self) -> Dict[str, Any]:
        return self._post_lims("/api/lims/scheduler/start")
    # 3.10 停止调度
    def scheduler_stop(self) -> Dict[str, Any]:

        """
        停止调度 (3.10)
        请求体只包含 apiKey 和 requestTime
        """
        return self._post_lims("/api/lims/scheduler/stop")
            # 2.9 继续调度
    # 2.9 继续调度
    def scheduler_continue(self) -> Dict[str, Any]:
        """
        继续调度 (2.9)
        请求体只包含 apiKey 和 requestTime
        """
        return self._post_lims("/api/lims/scheduler/continue")

    # 2.24 物料变更推送
    def report_material_change(self, material_obj: Dict[str, Any]) -> Dict[str, Any]:
        """
        material_obj 按 2.24 的裸对象格式（包含 id/typeName/locations/detail 等）
        """
        return self._post_report_raw("/report/material_change", material_obj)

    # 2.32 3-2-1 物料转运
    def transfer_3_to_2_to_1(self,
                            #  source_wh_id: Optional[str] = None,
                            source_wh_id: Optional[str] = '3a19debc-84b4-0359-e2d4-b3beea49348b',
                             source_x: int = 1, source_y: int = 1, source_z: int = 1) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "sourcePosX": source_x, "sourcePosY": source_y, "sourcePosZ": source_z
        }
        if source_wh_id:
            payload["sourceWHID"] = source_wh_id
        return self._post_lims("/api/lims/order/transfer-task3To2To1", payload)

    # 3.35 1→2 物料转运
    def transfer_1_to_2(self) -> Dict[str, Any]:
        """
        1→2 物料转运
        URL: /api/lims/order/transfer-task1To2
        只需要 apiKey 和 requestTime
        """
        return self._post_lims("/api/lims/order/transfer-task1To2")
   
    # 2.5 批量查询实验报告(post过滤关键字查询)
    def order_list_v2(self,
                      timeType: str = "",
                      beginTime: str = "",
                      endTime: str = "",
                      status: str = "", # 60表示正在运行,80表示完成，90表示失败
                      filter: str = "",
                      skipCount: int = 0,
                      pageCount: int = 1, # 显示多少页数据
                      sorting: str = "") -> Dict[str, Any]:
        """
        批量查询实验报告的详细信息 (2.5)
        URL: /api/lims/order/order-list
        参数默认值和接口文档保持一致
        """
        data: Dict[str, Any] = {
            "timeType": timeType,
            "beginTime": beginTime,
            "endTime": endTime,
            "status": status,
            "filter": filter,
            "skipCount": skipCount,
            "pageCount": pageCount,
            "sorting": sorting
        }
        return self._post_lims("/api/lims/order/order-list", data)

    # 一直post执行bioyond接口查询任务状态
    def wait_for_transfer_task(self, timeout: int = 3000, interval: int = 5, filter_text: Optional[str] = None) -> bool:
        """
        轮询查询物料转移任务是否成功完成 (status=80)
        - timeout: 最大等待秒数 (默认600秒)
        - interval: 轮询间隔秒数 (默认3秒)
        返回 True 表示找到并成功完成，False 表示超时未找到
        """
        now = datetime.now()
        beginTime = now.strftime("%Y-%m-%dT%H:%M:%SZ")
        endTime = (now + timedelta(minutes=5)).strftime("%Y-%m-%dT%H:%M:%SZ")
        print(beginTime, endTime)

        deadline = time.time() + timeout

        while time.time() < deadline:
            result = self.order_list_v2(
                timeType="",
                beginTime=beginTime,
                endTime=endTime,
                status="",
                filter=filter_text,
                skipCount=0,
                pageCount=1,
                sorting=""
            )
            print(result)

            items = result.get("data", {}).get("items", [])
            for item in items:
                name = item.get("name", "")
                status = item.get("status")
                # 改成用 filter_text 判断
                if (not filter_text or filter_text in name) and status == 80:
                    logger.info(f"硬件转移动作完成: {name}, status={status}")
                    return True

                logger.info(f"等待中: {name}, status={status}")
            time.sleep(interval)

        logger.warning("超时未找到成功的物料转移任务")
        return False

    def create_solid_materials(self, material_names: List[str], type_id: str = "3a190ca0-b2f6-9aeb-8067-547e72c11469") -> List[Dict[str, Any]]:
        """
        批量创建固体物料
        
        Args:
            material_names: 物料名称列表
            type_id: 物料类型ID（默认为固体物料类型）
            
        Returns:
            创建的物料列表，每个元素包含物料信息和ID
        """
        created_materials = []
        total = len(material_names)
        
        for i, name in enumerate(material_names, 1):
            # 根据接口文档构建完整的请求体
            material_data = {
                "typeId": type_id,
                "name": name,
                "unit": "g",              # 添加单位
                "quantity": 1,            # 添加数量（默认1）
                "parameters": ""          # 参数字段（空字符串表示无参数）
            }
            
            logger.info(f"正在创建第 {i}/{total} 个固体物料: {name}")
            result = self._post_lims("/api/lims/storage/material", material_data)
            
            if result and result.get("code") == 1:
                # data 字段可能是字符串（物料ID）或字典（包含id字段）
                data = result.get("data")
                if isinstance(data, str):
                    # data 直接是物料ID字符串
                    material_id = data
                elif isinstance(data, dict):
                    # data 是字典，包含id字段
                    material_id = data.get("id")
                else:
                    material_id = None
                
                if material_id:
                    created_materials.append({
                        "name": name,
                        "materialId": material_id,
                        "typeId": type_id
                    })
                    logger.info(f"✓ 成功创建物料: {name}, ID: {material_id}")
                else:
                    logger.error(f"✗ 创建物料失败: {name}, 未返回ID")
                    logger.error(f"  响应数据: {result}")
            else:
                error_msg = result.get("error") or result.get("message", "未知错误")
                logger.error(f"✗ 创建物料失败: {name}")
                logger.error(f"  错误信息: {error_msg}")
                logger.error(f"  完整响应: {result}")
                
            # 避免请求过快
            time.sleep(0.3)
        
        logger.info(f"物料创建完成，成功创建 {len(created_materials)}/{total} 个固体物料")
        return created_materials

    def create_and_inbound_materials_from_csv(
        self, 
        csv_path: str = "solid_materials.csv",
        type_id: str = "3a190ca0-b2f6-9aeb-8067-547e72c11469",
        warehouse_name: str = "粉末加样头堆栈"
    ) -> Dict[str, Any]:
        """
        从CSV文件读取物料列表，创建物料并批量入库到指定堆栈
        
        Args:
            csv_path: CSV文件路径
            type_id: 物料类型ID（默认为固体物料类型）
            warehouse_name: 仓库名称（默认为"粉末加样头堆栈"）
            
        Returns:
            包含执行结果的字典
        """
        logger.info("=" * 60)
        logger.info(f"开始执行：从CSV读取物料列表并批量创建入库到 {warehouse_name}")
        logger.info("=" * 60)
        
        # 从配置中获取位置ID列表
        warehouse_mapping = self.bioyond_config.get("warehouse_mapping", WAREHOUSE_MAPPING)
        
        if warehouse_name not in warehouse_mapping:
            error_msg = f"配置中未找到仓库: {warehouse_name}"
            logger.error(error_msg)
            logger.info(f"可用的仓库: {list(warehouse_mapping.keys())}")
            return {"success": False, "error": error_msg}
        
        warehouse_config = warehouse_mapping[warehouse_name]
        site_uuids = warehouse_config.get("site_uuids", {})
        
        if not site_uuids:
            error_msg = f"仓库 {warehouse_name} 没有配置位置"
            logger.error(error_msg)
            return {"success": False, "error": error_msg}
        
        # 按顺序获取位置ID（A01, B01, C01...）
        all_location_ids = []
        position_names = []
        for key in sorted(site_uuids.keys()):
            all_location_ids.append(site_uuids[key])
            position_names.append(key)
        
        logger.info(f"✓ 从配置文件加载 {len(all_location_ids)} 个位置")
        logger.info(f"  仓库: {warehouse_name}")
        logger.info(f"  位置范围: {position_names[0]} ~ {position_names[-1]}")
        
        # 读取CSV文件
        csv_file_path = Path(csv_path)
        material_names = []
        
        try:
            df_materials = pd.read_csv(csv_file_path)
            if 'material_name' in df_materials.columns:
                material_names = df_materials['material_name'].dropna().astype(str).str.strip().tolist()
                logger.info(f"✓ 成功从CSV文件读取 {len(material_names)} 个物料名称")
                logger.info(f"  文件路径: {csv_file_path}")
            else:
                logger.error(f"✗ CSV文件缺少 'material_name' 列")
                return {"success": False, "error": "CSV文件缺少 'material_name' 列"}
        except FileNotFoundError:
            logger.error(f"✗ 未找到CSV文件: {csv_file_path}")
            logger.info("请创建CSV文件，格式：")
            logger.info("  material_name")
            logger.info("  LiPF6")
            logger.info("  LiDFOB")
            logger.info("  ...")
            return {"success": False, "error": f"未找到CSV文件: {csv_file_path}"}
        except Exception as e:
            logger.error(f"✗ 读取CSV文件失败: {e}")
            return {"success": False, "error": f"读取CSV文件失败: {e}"}
        
        if not material_names:
            logger.error("CSV文件中没有有效的物料名称")
            return {"success": False, "error": "CSV文件中没有有效的物料名称"}
        
        # 检查物料数量
        if len(material_names) > len(all_location_ids):
            logger.warning(f"物料数量({len(material_names)})超过可用位置数量({len(all_location_ids)})！")
            logger.warning(f"将仅创建前 {len(all_location_ids)} 个物料")
            material_names = material_names[:len(all_location_ids)]
        
        # 准备位置信息
        location_ids = all_location_ids[:len(material_names)]
        selected_positions = position_names[:len(material_names)]
        
        # 步骤1: 创建固体物料
        logger.info(f"\n【步骤1/2】创建 {len(material_names)} 个固体物料...")
        logger.info(f"物料类型ID: {type_id}")
        logger.info(f"物料列表: {', '.join(material_names)}")
        
        created_materials = self.create_solid_materials(
            material_names=material_names,
            type_id=type_id
        )
        
        if len(created_materials) != len(material_names):
            logger.warning(f"创建的物料数量({len(created_materials)})与计划数量({len(material_names)})不匹配！")
            logger.warning("将仅对成功创建的物料进行入库操作")
        
        if not created_materials:
            logger.error("没有成功创建任何物料")
            return {"success": False, "error": "没有成功创建任何物料"}
        
        # 步骤2: 批量入库到指定位置
        logger.info(f"\n【步骤2/2】批量入库物料到 {warehouse_name}...")
        inbound_items = []
        
        for idx, material in enumerate(created_materials):
            if idx < len(location_ids):
                inbound_items.append({
                    "materialId": material["materialId"],
                    "locationId": location_ids[idx]
                })
                logger.info(f"  - {material['name']} (ID: {material['materialId'][:8]}...) → 位置 {selected_positions[idx]}")
        
        logger.info(f"\n正在执行批量入库，共 {len(inbound_items)} 条记录...")
        result = self.storage_batch_inbound(inbound_items)
        
        inbound_success = result.get("code") == 1
        
        if inbound_success:
            logger.info(f"✓ 批量入库成功！")
            logger.info(f"  响应数据: {result.get('data', {})}")
            
            # 步骤3: 同步物料数据
            logger.info(f"\n【步骤3/3】同步物料数据到系统...")
            if hasattr(self, 'resource_synchronizer') and self.resource_synchronizer:
                try:
                    # 尝试同步不同类型的物料
                    # typeMode: 0=耗材, 1=样品, 2=试剂
                    sync_success = False
                    for type_mode in [0, 1, 2]:
                        try:
                            logger.info(f"  尝试同步 typeMode={type_mode} 的物料...")
                            bioyond_data = self.hardware_interface.stock_material(
                                f'{{"typeMode": {type_mode}, "includeDetail": true}}'
                            )
                            if bioyond_data:
                                logger.info(f"  ✓ 获取到 {len(bioyond_data) if isinstance(bioyond_data, list) else 1} 条物料数据")
                                sync_success = True
                        except Exception as e:
                            logger.debug(f"  typeMode={type_mode} 同步失败: {e}")
                            continue
                    
                    if sync_success:
                        logger.info(f"✓ 物料数据同步完成")
                    else:
                        logger.warning(f"⚠ 物料数据同步未获取到数据（这是正常的，新创建的物料可能需要时间才能查询到）")
                        
                except Exception as e:
                    logger.warning(f"⚠ 物料数据同步出错: {e}")
                    logger.info(f"  提示：新创建的物料已成功入库，同步失败不影响使用")
            else:
                logger.warning("⚠ 资源同步器未初始化，跳过同步")
        else:
            logger.error(f"✗ 批量入库失败！")
            logger.error(f"  响应: {result}")
        
        logger.info("\n" + "=" * 60)
        logger.info("固体物料创建和入库流程完成")
        logger.info("=" * 60 + "\n")
        
        return {
            "success": inbound_success,
            "created_materials": created_materials,
            "inbound_result": result,
            "total_created": len(created_materials),
            "total_inbound": len(inbound_items),
            "warehouse": warehouse_name,
            "positions": selected_positions
        }


# --------------------------------


if __name__ == "__main__":
    ws = BioyondCellWorkstation()
    logger.info(ws.scheduler_start())
    
    # 从CSV文件读取物料列表并批量创建入库
    result = ws.create_and_inbound_materials_from_csv()
    
    # 继续后续流程
    logger.info(ws.auto_feeding4to3()) #搬运物料到3号箱
    # 使用正斜杠或 Path 对象来指定文件路径
    excel_path = Path("unilabos/devices/workstation/bioyond_studio/bioyond_cell/2025092701.xlsx")
    logger.info(ws.create_orders(excel_path))
    logger.info(ws.transfer_3_to_2_to_1())

    logger.info(ws.transfer_1_to_2())



    while True:
        time.sleep(1)
    # re=ws.scheduler_stop()
    # re = ws.transfer_3_to_2_to_1()

    # print(re)
    # logger.info("调度启动完成")

    # ws.scheduler_continue()
    # 3.30 上料：读取模板 Excel 自动解析并 POST
    # r1 = ws.auto_feeding4to3_from_xlsx(r"C:\ML\GitHub\Uni-Lab-OS\unilabos\devices\workstation\bioyond_cell\样品导入模板.xlsx")
    # ws.wait_for_transfer_task(filter_text="物料转移任务")
    # logger.info("4号箱向3号箱转运物料转移任务已完成")

    # ws.scheduler_start()
    # print(r1["payload"]["data"])   # 调试模式下可直接看到要发的 JSON items

    # # 新建实验
    # response = ws.create_orders("C:/ML/GitHub/Uni-Lab-OS/unilabos/devices/workstation/bioyond_cell/2025092701.xlsx")
    # logger.info(response)
    # data_list = response.get("data", [])
    # order_name = data_list[0].get("orderName", "")

    # ws.wait_for_transfer_task(filter_text=order_name)
    # ws.wait_for_transfer_task(filter_text='DP20250927001')
    # logger.info("3号站内实验完成")
    # # ws.scheduler_start()
    # # print(res)
    # ws.transfer_3_to_2_to_1()
    # ws.wait_for_transfer_task(filter_text="物料转移任务")
    # logger.info("3号站向2号站向1号站转移任务完成")
        # r321 = self.wait_for_transfer_task()
    #1号站启动
    # ws.transfer_1_to_2()
    # ws.wait_for_transfer_task(filter_text="物料转移任务")
    # logger.info("1号站向2号站转移任务完成")
    # logger.info("全流程结束")

    # 3.31 下料：同理
    # r2 = ws.auto_batch_outbound_from_xlsx(r"C:/path/样品导入模板 (8).xlsx")
    # print(r2["payload"]["data"])
