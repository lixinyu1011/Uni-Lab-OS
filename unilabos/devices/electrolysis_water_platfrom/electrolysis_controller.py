# -*- coding: utf-8 -*-
"""
电解水平台控制器
接收传感器数据，通过深度学习模型计算控制向量，返回给平台执行

数据流：
    平台解析传感器数据 → 控制器接收数据 → 深度学习模型计算 → 返回控制向量 → 平台执行
"""
import numpy as np
import time
import threading
from typing import Optional, Dict, Any, List
from pathlib import Path

from unilabos.devices.workstation.workstation_base import WorkstationBase


class ElectrolysisController(WorkstationBase):
    """
    电解水平台控制器
    
    负责接收传感器数据，通过深度学习模型计算控制参数，返回控制向量
    """
    
    def __init__(
        self,
        model_path: Optional[str] = None,
        model_type: str = "pytorch",  # 'pytorch' 或 'tensorflow' 或 'dummy'
        input_dim: int = 6,
        output_dim: int = 5,
        device: str = "cpu",
        deck=None,  # WorkstationBase 需要的 deck 参数
        *args,
        **kwargs
    ):
        """
        初始化控制器
        
        Args:
            model_path: 模型文件路径（可选，如果不提供则使用简单规则）
            model_type: 模型类型 ('pytorch', 'tensorflow', 'dummy')
            input_dim: 输入维度（默认6：电流、电压、温度、pH、气流、液流）
            output_dim: 输出维度（默认5：模式、电流、电压、温度、泵速）
            device: 计算设备 ('cpu' 或 'cuda')
            deck: 工作站deck（可选，控制器作为虚拟工作站不需要物理deck）
        """
        # 初始化父类 WorkstationBase
        super().__init__(deck=deck, *args, **kwargs)
        
        self.model_path = Path(model_path) if model_path else None
        self.model_type = model_type.lower()
        self.device = device
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.model = None
        
        # 数据归一化参数
        self.input_mean = None
        self.input_std = None
        self.output_mean = None
        self.output_std = None
        
        # 演示模式相关
        self.demo_mode = True  # 默认开启演示模式
        self.demo_thread: Optional[threading.Thread] = None
        self.demo_running = False
        self.demo_interval = 2.0  # 演示数据发送间隔（秒）
        
        # 接收数据统计
        self.data_received_count = 0
        self.data_sent_count = 0
        
        # 加载模型（虚拟演示）
        if self.model_path and self.model_path.exists():
            self._load_model()
            print(f"[Controller] ✓ 模型已加载: {self.model_path}")
        else:
            # 虚拟模型加载演示
            print(f"[Controller] 🤖 虚拟深度学习模型初始化中...")
            time.sleep(0.5)
            print(f"[Controller] ✓ 虚拟模型加载成功！")
            print(f"[Controller]   - 模型类型: {self.model_type}")
            print(f"[Controller]   - 输入维度: {self.input_dim} (电流/电压/温度/pH/气流/液流)")
            print(f"[Controller]   - 输出维度: {self.output_dim} (模式/电流/电压/温度/泵速)")
            print(f"[Controller]   - 计算设备: {self.device}")
            print(f"[Controller]   - 演示模式: {'开启' if self.demo_mode else '关闭'}")
    
    def _load_model(self):
        """加载深度学习模型"""
        if self.model_type == "pytorch":
            self._load_pytorch_model()
        elif self.model_type == "tensorflow":
            self._load_tensorflow_model()
        else:
            print(f"[Controller] 不支持的模型类型: {self.model_type}")
    
    def _load_pytorch_model(self):
        """加载 PyTorch 模型"""
        try:
            import torch
            self.model = torch.load(self.model_path, map_location=self.device)
            self.model.eval()
            print(f"[Controller] PyTorch 模型加载成功")
        except ImportError:
            print("[Controller] 错误: 需要安装 PyTorch")
        except Exception as e:
            print(f"[Controller] 加载 PyTorch 模型失败: {e}")
    
    def _load_tensorflow_model(self):
        """加载 TensorFlow 模型"""
        try:
            import tensorflow as tf
            self.model = tf.keras.models.load_model(str(self.model_path))
            print(f"[Controller] TensorFlow 模型加载成功")
        except ImportError:
            print("[Controller] 错误: 需要安装 TensorFlow")
        except Exception as e:
            print(f"[Controller] 加载 TensorFlow 模型失败: {e}")
    
    def set_normalization_params(
        self,
        input_mean: List[float],
        input_std: List[float],
        output_mean: List[float],
        output_std: List[float]
    ):
        """
        设置数据归一化参数
        
        Args:
            input_mean: 输入均值
            input_std: 输入标准差
            output_mean: 输出均值
            output_std: 输出标准差
        """
        self.input_mean = np.array(input_mean, dtype=np.float32)
        self.input_std = np.array(input_std, dtype=np.float32)
        self.output_mean = np.array(output_mean, dtype=np.float32)
        self.output_std = np.array(output_std, dtype=np.float32)
        print("[Controller] 归一化参数已设置")
    
    def compute_control(self, sensor_data: List[float]) -> List[float]:
        """
        接收传感器数据，计算控制向量
        
        Args:
            sensor_data: 传感器数据列表 [current_mA, voltage_mV, temperature_C, ph, gas_flow, liquid_flow]
        
        Returns:
            控制向量列表 [mode, current_ma, voltage_mv, temperature_c, pump_percent]
        """
        self.data_received_count += 1
        
        # 演示模式：打印接收到的数据
        if self.demo_mode:
            print("\n" + "="*70)
            print(f"[Controller] 📥 接收数据 #{self.data_received_count}")
            print("-"*70)
            if len(sensor_data) >= 6:
                print(f"  电流:     {sensor_data[0]:>8.1f} mA")
                print(f"  电压:     {sensor_data[1]:>8.1f} mV")
                print(f"  温度:     {sensor_data[2]:>8.2f} °C")
                print(f"  pH值:     {sensor_data[3]:>8.2f}")
                print(f"  气体流量: {sensor_data[4]:>8.1f} sccm")
                print(f"  液体流量: {sensor_data[5]:>8.1f} mL")
            else:
                print(f"  原始数据: {sensor_data}")
        
        # 确保输入是正确的维度
        if len(sensor_data) < self.input_dim:
            if self.demo_mode:
                print(f"[Controller] ⚠️  数据维度不足，自动补齐 (期望{self.input_dim}维，实际{len(sensor_data)}维)")
            sensor_data = list(sensor_data) + [0.0] * (self.input_dim - len(sensor_data))
        
        # 演示：模拟模型推理过程
        if self.demo_mode:
            print("-"*70)
            print(f"[Controller] 🧠 深度学习模型推理中...")
            print(f"  模型类型: {self.model_type.upper()}")
            time.sleep(0.1)  # 模拟推理延迟
        
        # 转换为 numpy 数组
        input_vector = np.array(sensor_data[:self.input_dim], dtype=np.float32).reshape(1, -1)
        
        # 数据归一化
        if self.input_mean is not None and self.input_std is not None:
            input_vector = (input_vector - self.input_mean) / (self.input_std + 1e-8)
            if self.demo_mode:
                print(f"  数据归一化: ✓")
        
        # 模型推理（生成固定的演示控制向量）
        if self.model is not None:
            if self.model_type == "pytorch":
                output_vector = self._predict_pytorch(input_vector)
            elif self.model_type == "tensorflow":
                output_vector = self._predict_tensorflow(input_vector)
            else:
                output_vector = self._demo_control(sensor_data)
        else:
            # 使用演示控制规则
            output_vector = self._demo_control(sensor_data)
        
        # 反归一化
        if self.output_mean is not None and self.output_std is not None:
            output_vector = output_vector * self.output_std + self.output_mean
        
        # 转换为列表并返回
        control_vector = output_vector.flatten().tolist()
        
        # 确保输出维度正确
        if len(control_vector) < self.output_dim:
            control_vector = control_vector + [0.0] * (self.output_dim - len(control_vector))
        
        control_vector = control_vector[:self.output_dim]
        
        # 演示模式：打印输出的控制向量
        if self.demo_mode:
            print(f"  推理完成: ✓")
            print("-"*70)
            print(f"[Controller] 📤 发送控制向量 #{self.data_sent_count + 1}")
            print("-"*70)
            mode_str = "恒压模式" if int(control_vector[0]) == 0 else "恒流模式"
            print(f"  控制模式: {mode_str}")
            print(f"  目标电流: {control_vector[1]:>8.1f} mA")
            print(f"  目标电压: {control_vector[2]:>8.1f} mV")
            print(f"  目标温度: {control_vector[3]:>8.2f} °C")
            print(f"  泵速设置: {control_vector[4]:>8.1f} %")
            print("="*70 + "\n")
        
        self.data_sent_count += 1
        
        return control_vector
    
    def _predict_pytorch(self, input_vector: np.ndarray) -> np.ndarray:
        """PyTorch 模型推理"""
        try:
            import torch
            with torch.no_grad():
                input_tensor = torch.from_numpy(input_vector).to(self.device)
                output_tensor = self.model(input_tensor)
                output_vector = output_tensor.cpu().numpy()
            return output_vector
        except Exception as e:
            print(f"[Controller] PyTorch 推理失败: {e}")
            return self._simple_control(input_vector.flatten().tolist())
    
    def _predict_tensorflow(self, input_vector: np.ndarray) -> np.ndarray:
        """TensorFlow 模型推理"""
        try:
            output_vector = self.model.predict(input_vector, verbose=0)
            return output_vector
        except Exception as e:
            print(f"[Controller] TensorFlow 推理失败: {e}")
            return self._simple_control(input_vector.flatten().tolist())
    
    def _simple_control(self, sensor_data: List[float]) -> np.ndarray:
        """
        简单控制规则（当没有深度学习模型时使用）
        
        实现简单的 PID 控制逻辑
        """
        # 提取传感器数据
        current_mA = sensor_data[0] if len(sensor_data) > 0 else 0
        voltage_mV = sensor_data[1] if len(sensor_data) > 1 else 0
        temperature_C = sensor_data[2] if len(sensor_data) > 2 else 25.0
        
        # 简单控制规则：
        # 1. 如果电压低于目标，增加电压
        # 2. 如果温度过高，降低电流
        target_voltage = 2500  # 目标电压 2.5V
        target_current = 1000  # 目标电流 1A
        
        # 电压控制（简单比例控制）
        voltage_error = target_voltage - voltage_mV
        control_voltage = target_voltage + 0.5 * voltage_error
        
        # 电流控制
        control_current = target_current
        
        # 温度保护
        if temperature_C > 30.0:
            control_current = max(500, control_current - 200)
        
        # 限幅
        control_voltage = max(0, min(5000, control_voltage))
        control_current = max(0, min(2000, control_current))
        
        # 返回控制向量：[mode, current_ma, voltage_mv, temperature_c, pump_percent]
        control_vector = np.array([
            0,                  # mode: 0=恒压
            control_current,    # 电流
            control_voltage,    # 电压
            25.0,              # 温度
            50.0               # 泵速
        ], dtype=np.float32).reshape(1, -1)
        
        return control_vector
    
    def _demo_control(self, sensor_data: List[float]) -> np.ndarray:
        """
        演示控制规则（返回固定控制值用于演示）
        
        根据当前传感器数据，返回固定的优化控制参数
        """
        # 演示：返回固定的优化控制值
        # 这些值可以调整电解水平台的运行状态
        
        # 固定控制策略
        control_vector = np.array([
            0,        # mode: 0=恒压模式
            1200,     # 目标电流: 1200mA (比默认值稍高，提升效率)
            2800,     # 目标电压: 2800mV (2.8V，优化电解效率)
            28.0,     # 目标温度: 28°C (略高于室温，提升反应速率)
            65.0      # 泵速: 65% (提升循环效率)
        ], dtype=np.float32).reshape(1, -1)
        
        return control_vector
    
    def start_demo_loop(self, interval: float = 2.0):
        """
        启动演示循环，持续接收和发送虚拟数据
        
        Args:
            interval: 数据发送间隔（秒）
        """
        if self.demo_running:
            print("[Controller] ⚠️  演示循环已在运行")
            return
        
        self.demo_interval = interval
        self.demo_running = True
        self.demo_thread = threading.Thread(
            target=self._demo_loop,
            daemon=True,
            name="controller_demo"
        )
        self.demo_thread.start()
        print(f"[Controller] ✓ 演示循环已启动（间隔: {interval}秒）")
    
    def stop_demo_loop(self):
        """停止演示循环"""
        if not self.demo_running:
            return
        
        self.demo_running = False
        if self.demo_thread and self.demo_thread.is_alive():
            self.demo_thread.join(timeout=2.0)
        print(f"[Controller] ✓ 演示循环已停止")
        print(f"[Controller]   - 总接收数据: {self.data_received_count} 次")
        print(f"[Controller]   - 总发送控制: {self.data_sent_count} 次")
    
    def _demo_loop(self):
        """演示循环主函数"""
        print(f"\n[Controller] 🚀 开始演示数据流...")
        
        while self.demo_running:
            # 模拟从平台接收传感器数据
            virtual_sensor_data = [
                1000.0 + np.random.uniform(-100, 100),  # 电流 (mA)
                2500.0 + np.random.uniform(-200, 200),  # 电压 (mV)
                25.0 + np.random.uniform(-2, 2),        # 温度 (°C)
                7.0 + np.random.uniform(-0.5, 0.5),     # pH
                50.0 + np.random.uniform(-10, 10),      # 气体流量 (sccm)
                30.0 + np.random.uniform(-5, 5)         # 液体流量 (mL)
            ]
            
            # 计算控制向量并打印
            control_vector = self.compute_control(virtual_sensor_data)
            
            # 等待下一个周期
            time.sleep(self.demo_interval)
        
        print(f"[Controller] 演示循环结束")


# ==================== 示例函数 ====================

def create_simple_pytorch_model(input_dim=6, hidden_dim=32, output_dim=5):
    """创建一个简单的 PyTorch 模型示例"""
    try:
        import torch
        import torch.nn as nn
        
        class ControlNet(nn.Module):
            def __init__(self):
                super().__init__()
                self.network = nn.Sequential(
                    nn.Linear(input_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, output_dim)
                )
            
            def forward(self, x):
                return self.network(x)
        
        model = ControlNet()
        return model
    except ImportError:
        print("需要安装 PyTorch: pip install torch")
        return None


def create_simple_tensorflow_model(input_dim=6, hidden_dim=32, output_dim=5):
    """创建一个简单的 TensorFlow 模型示例"""
    try:
        import tensorflow as tf
        
        model = tf.keras.Sequential([
            tf.keras.layers.Dense(hidden_dim, activation='relu', input_shape=(input_dim,)),
            tf.keras.layers.Dense(hidden_dim, activation='relu'),
            tf.keras.layers.Dense(output_dim)
        ])
        
        model.compile(optimizer='adam', loss='mse')
        return model
    except ImportError:
        print("需要安装 TensorFlow: pip install tensorflow")
        return None


# ==================== 使用示例 ====================

if __name__ == "__main__":
    print("="*70)
    print("电解水平台控制器 - 演示模式")
    print("="*70)
    
    # 示例 1: 创建控制器并运行演示循环
    print("\n【示例 1】创建虚拟AI控制器并启动演示")
    controller = ElectrolysisController(
        model_type="pytorch",
        input_dim=6,
        output_dim=5
    )
    
    # 启动演示循环（持续5次数据交换）
    print("\n[提示] 演示将持续约10秒，展示数据接收和控制发送...")
    controller.start_demo_loop(interval=2.0)
    
    # 让演示运行一段时间
    try:
        time.sleep(11)  # 运行11秒
    except KeyboardInterrupt:
        print("\n用户中断")
    
    # 停止演示
    controller.stop_demo_loop()
    
    # 示例 2: 手动测试单次控制计算
    print("\n" + "="*70)
    print("【示例 2】手动测试单次数据处理")
    print("="*70)
    
    # 创建不带演示打印的控制器
    controller2 = ElectrolysisController(model_type="pytorch")
    controller2.demo_mode = False  # 关闭详细打印
    
    sensor_data = [
        1050.0,  # 电流 (mA)
        2300.0,  # 电压 (mV)
        26.5,    # 温度 (°C)
        7.2,     # pH
        48.0,    # 气体流量 (sccm)
        32.0     # 液体流量 (mL)
    ]
    
    print(f"输入传感器数据: {sensor_data}")
    control_vector = controller2.compute_control(sensor_data)
    print(f"输出控制向量: {control_vector}")
    
    print("\n" + "="*70)
    print("演示完成！")
    print("="*70)
