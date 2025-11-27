# -*- coding: utf-8 -*-
"""
电解水平台控制器
接收传感器数据，通过深度学习模型计算控制向量，返回给平台执行

数据流：
    平台解析传感器数据 → 控制器接收数据 → 深度学习模型计算 → 返回控制向量 → 平台执行
"""
import numpy as np
from typing import Optional, Dict, Any, List
from pathlib import Path


class ElectrolysisController:
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
        device: str = "cpu"
    ):
        """
        初始化控制器
        
        Args:
            model_path: 模型文件路径（可选，如果不提供则使用简单规则）
            model_type: 模型类型 ('pytorch', 'tensorflow', 'dummy')
            input_dim: 输入维度（默认6：电流、电压、温度、pH、气流、液流）
            output_dim: 输出维度（默认5：模式、电流、电压、温度、泵速）
            device: 计算设备 ('cpu' 或 'cuda')
        """
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
        
        # 加载模型
        if self.model_path and self.model_path.exists():
            self._load_model()
            print(f"[Controller] 模型已加载: {self.model_path}")
        else:
            print(f"[Controller] 未加载模型，使用简单控制规则")
    
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
        # 确保输入是正确的维度
        if len(sensor_data) < self.input_dim:
            print(f"[Controller] 警告: 输入数据维度不足，期望 {self.input_dim}，实际 {len(sensor_data)}")
            # 补齐数据
            sensor_data = list(sensor_data) + [0.0] * (self.input_dim - len(sensor_data))
        
        # 转换为 numpy 数组
        input_vector = np.array(sensor_data[:self.input_dim], dtype=np.float32).reshape(1, -1)
        
        # 数据归一化
        if self.input_mean is not None and self.input_std is not None:
            input_vector = (input_vector - self.input_mean) / (self.input_std + 1e-8)
        
        # 模型推理
        if self.model is not None:
            if self.model_type == "pytorch":
                output_vector = self._predict_pytorch(input_vector)
            elif self.model_type == "tensorflow":
                output_vector = self._predict_tensorflow(input_vector)
            else:
                output_vector = self._simple_control(sensor_data)
        else:
            # 没有模型，使用简单控制规则
            output_vector = self._simple_control(sensor_data)
        
        # 反归一化
        if self.output_mean is not None and self.output_std is not None:
            output_vector = output_vector * self.output_std + self.output_mean
        
        # 转换为列表并返回
        control_vector = output_vector.flatten().tolist()
        
        # 确保输出维度正确
        if len(control_vector) < self.output_dim:
            control_vector = control_vector + [0.0] * (self.output_dim - len(control_vector))
        
        return control_vector[:self.output_dim]
    
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
    print("电解水平台控制器示例")
    print("="*70)
    
    # 示例 1: 创建控制器（无模型，使用简单规则）
    print("\n【示例 1】创建控制器（无模型）")
    controller = ElectrolysisController()
    
    # 模拟传感器数据
    sensor_data = [
        1000.0,  # 电流 (mA)
        2000.0,  # 电压 (mV)
        25.0,    # 温度 (°C)
        7.0,     # pH
        50.0,    # 气体流量 (sccm)
        30.0     # 液体流量 (mL)
    ]
    
    print(f"输入传感器数据: {sensor_data}")
    
    # 计算控制向量
    control_vector = controller.compute_control(sensor_data)
    print(f"输出控制向量: {control_vector}")
    print(f"  - 模式: {int(control_vector[0])} ({'恒压' if int(control_vector[0]) == 0 else '恒流'})")
    print(f"  - 电流: {control_vector[1]:.1f} mA")
    print(f"  - 电压: {control_vector[2]:.1f} mV")
    print(f"  - 温度: {control_vector[3]:.1f} °C")
    print(f"  - 泵速: {control_vector[4]:.1f} %")
    
    # 示例 2: 创建并保存模型
    print("\n【示例 2】创建并保存 PyTorch 模型")
    try:
        import torch
        model = create_simple_pytorch_model()
        if model:
            torch.save(model, "electrolysis_model.pth")
            print("✓ 模型已保存: electrolysis_model.pth")
            
            # 使用保存的模型创建控制器
            print("\n【示例 3】使用模型创建控制器")
            controller_with_model = ElectrolysisController(
                model_path="electrolysis_model.pth",
                model_type="pytorch"
            )
            
            # 测试
            control_vector = controller_with_model.compute_control(sensor_data)
            print(f"输出控制向量: {control_vector}")
    except ImportError:
        print("⚠ PyTorch 未安装，跳过此示例")
    
    # 示例 3: 设置归一化参数
    print("\n【示例 4】设置归一化参数")
    controller.set_normalization_params(
        input_mean=[1000, 2500, 25, 7, 50, 30],
        input_std=[500, 1000, 5, 2, 25, 15],
        output_mean=[0, 1000, 2500, 25, 50],
        output_std=[1, 500, 1000, 5, 25]
    )
    
    control_vector = controller.compute_control(sensor_data)
    print(f"使用归一化后的输出: {control_vector}")
    
    print("\n" + "="*70)
    print("示例完成！")
    print("="*70)
