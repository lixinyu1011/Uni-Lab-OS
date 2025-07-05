import socket, json, contextlib
from typing import Any, List, Dict, Optional


class PRCXIError(RuntimeError):
    """Lilith 返回 Success=false 时抛出的业务异常"""


class PRCXI9300:

    def __init__(self, host: str = "127.0.0.1", port: int = 9999,
                 timeout: float = 10.0) -> None:
        self.host, self.port, self.timeout = host, port, timeout


    @staticmethod
    def _len_prefix(n: int) -> bytes:
        return bytes.fromhex(format(n, "016x"))

    def _raw_request(self, payload: str) -> str:
        with contextlib.closing(socket.socket()) as sock:
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.port))
            data = payload.encode()
            sock.sendall(self._len_prefix(len(data)) + data)

            chunks, first = [], True
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                if first:
                    chunk, first = chunk[8:], False  
                chunks.append(chunk)
            return b"".join(chunks).decode()

    def _call(self, service: str, method: str,
              params: Optional[list] = None) -> Any:
        payload = json.dumps(
            {"ServiceName": service,
             "MethodName": method,
             "Paramters": params or []},
            separators=(",", ":")
        )
        resp = json.loads(self._raw_request(payload))
        if not resp.get("Success", False):
            raise PRCXIError(resp.get("Msg", "Unknown error"))
        data = resp.get("Data")
        try:
            return json.loads(data)
        except (TypeError, json.JSONDecodeError):
            return data

    # ---------------------------------------------------- 方案相关（ISolution）
    def list_solutions(self) -> List[Dict[str, Any]]:
        """GetSolutionList"""
        return self._call("ISolution", "GetSolutionList")

    def load_solution(self, solution_id: str) -> bool:
        """LoadSolution"""
        return self._call("ISolution", "LoadSolution", [solution_id])

    def add_solution(self, name: str, matrix_id: str,
                     steps: List[Dict[str, Any]]) -> str:
        """AddSolution → 返回新方案 GUID"""
        return self._call("ISolution", "AddSolution",
                          [name, matrix_id, steps])

    # ---------------------------------------------------- 自动化控制（IAutomation）
    def start(self) -> bool:
        return self._call("IAutomation", "Start")

    def stop(self) -> bool:
        """Stop"""
        return self._call("IAutomation", "Stop")

    def reset(self) -> bool:
        """Reset"""
        return self._call("IAutomation", "Reset")

    def pause(self) -> bool:
        """Pause"""
        return self._call("IAutomation", "Pause")

    def resume(self) -> bool:
        """Resume"""
        return self._call("IAutomation", "Resume")

    def get_error_code(self) -> Optional[str]:
        """GetErrorCode"""
        return self._call("IAutomation", "GetErrorCode")

    def clear_error_code(self) -> bool:
        """RemoveErrorCodet"""
        return self._call("IAutomation", "RemoveErrorCodet")

    # ---------------------------------------------------- 运行状态（IMachineState）
    def step_state_list(self) -> List[Dict[str, Any]]:
        """GetStepStateList"""
        return self._call("IMachineState", "GetStepStateList")

    def step_status(self, seq_num: int) -> Dict[str, Any]:
        """GetStepStatus"""
        return self._call("IMachineState", "GetStepStatus", [seq_num])

    def step_state(self, seq_num: int) -> Dict[str, Any]:
        """GetStepState"""
        return self._call("IMachineState", "GetStepState", [seq_num])

    def axis_location(self, axis_num: int = 1) -> Dict[str, Any]:
        """GetLocation"""
        return self._call("IMachineState", "GetLocation", [axis_num])

    # ---------------------------------------------------- 版位矩阵（IMatrix）
    def list_matrices(self) -> List[Dict[str, Any]]:
        """GetWorkTabletMatrices"""
        return self._call("IMatrix", "GetWorkTabletMatrices")

    def matrix_by_id(self, matrix_id: str) -> Dict[str, Any]:
        """GetWorkTabletMatrixById"""
        return self._call("IMatrix", "GetWorkTabletMatrixById", [matrix_id])
    
    def add_WorkTablet_Matrix(self,matrix):
        return self._call("IMatrix", "AddWorkTabletMatrix", [matrix])

    # ---------------------------------------------------- 一键运行
    def run_solution(self, solution_id: str, channel_idx: int = 1) -> None:
        self.load_solution(solution_id)
        self.start(channel_idx)

# ---------------------------------------------------- 辅助类 StepData 工具
def build_step(
    axis: str,
    function: str,
    dosage: int,
    plate_no: int,
    is_whole_plate: bool,
    hole_row: int,
    hole_col: int,
    blending_times: int,
    balance_height: int,
    plate_or_hole: str,
    hole_numbers: str,
    assist_fun1: str = "",
    assist_fun2: str = "",
    assist_fun3: str = "",
    assist_fun4: str = "",
    assist_fun5: str = "",
    liquid_method: str = "NormalDispense"
) -> Dict[str, Any]:
    return {
        "StepAxis": axis,
        "Function": function,
        "DosageNum": dosage,
        "PlateNo": plate_no,
        "IsWholePlate": is_whole_plate,
        "HoleRow": hole_row,
        "HoleCol": hole_col,
        "BlendingTimes": blending_times,
        "BalanceHeight": balance_height,
        "PlateOrHoleNum": plate_or_hole,
        "AssistFun1": assist_fun1,
        "AssistFun2": assist_fun2,
        "AssistFun3": assist_fun3,
        "AssistFun4": assist_fun4,
        "AssistFun5": assist_fun5,
        "HoleNumbers": hole_numbers,
        "LiquidDispensingMethod": liquid_method
    }
