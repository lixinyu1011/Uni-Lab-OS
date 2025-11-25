# -*- coding: utf-8 -*-
"""
电解水平台控制器
实现基于深度学习模型的自动控制策略
"""
import time
import threading
import numpy as np
from typing import Optional, Dict, Any, Callable
from collections import deque
from pathlib import Path


class DeepLearningController:
    """基于深度学习模型的控制器"""
    
    def __init__(
        self,
        model_path: str,
        model_type: str = "pytorch",  # 'pytorch' 或 'tensorflow'
        input_features: list = None,
        output_features: list = None,
        device: str = "cpu"
    ):
        """
        初始化深度学习控制器
        
        Args:
            model_path: 模型文件路径
            model_type: 模型类型 ('pytorch' 或 'tensorflow')
            input_features: 输入特征列表
            output_features: 输出特征列表
            device: 计算设备 ('cpu' 或 'cuda')
        """
        self.model_path = Path(model_path)
        self.model_type = model_type.lower()
        self.device = device
        self.model = None
        
        # 默认输入特征：电流、电压、温度、pH、气体流量、液体流量
        self.input_features = input_features or [
            'current', 'voltage', 'temperature', 'ph', 'gas_flow', 'liquid_flow'
        ]
        
        # 默认输出特征：目标电压、目标电流
        self.output_features = output_features or [
            'control_voltage', 'control_current'
        ]
        
        # 数据归一化参数（需要根据训练数据设置）
        self.input_mean = None
        self.input_std = None
        self.output_mean = None
        self.output_std = None
        
        # 加载模型
        self._load_model()
    
    def _load_model(self):
        """加载深度学习模型"""
        if not self.model_path.exists():
            raise FileNotFoundError(f"模型文件不存在: {self.model_path}")
        
        print(f"[DL Controller] 正在加载模型: {self.model_path}")
        
        if self.model_type == "pytorch":
            self._load_pytorch_model()
        elif self.model_type == "tensorflow":
            self._load_tensorflow_model()
        else:
            raise ValueError(f"不支持的模型类型: {self.model_type}")
        
        print(f"[DL Controller] 模型加载成功")
    
    def _load_pytorch_model(self):
        """加载 PyTorch 模型"""
        try:
            import torch
            
            # 加载模型
            self.model = torch.load(self.model_path, map_location=self.device)
            self.model.eval()
            
            # 如果是 state_dict，需要先定义模型结构
            if isinstance(self.model, dict):
                print("[DL Controller] 检测到 state_dict，需要先定义模型结构")
                # 这里需要用户提供模型定义
                raise NotImplementedError("请先定义模型结构，然后加载 state_dict")
            
        except ImportError:
            raise ImportError("需要安装 PyTorch: pip install torch")
    
    def _load_tensorflow_model(self):
        """加载 TensorFlow 模型"""
        try:
            import tensorflow as tf
            
            # 加载模型
            self.model = tf.keras.models.load_model(str(self.model_path))
            
        except ImportError:
            raise ImportError("需要安装 TensorFlow: pip install tensorflow")
    
    def predict(self, input_data: Dict[str, float]) -> Dict[str, float]:
        """
        使用模型进行预测
        
        Args:
            input_data: 输入数据字典，包含传感器读数
            
        Returns:
            预测的控制输出字典
        """
        # 提取输入特征
        input_vector = np.array([
            input_data.get(feature, 0.0) for feature in self.input_features
        ], dtype=np.float32).reshape(1, -1)
        
        # 数据归一化
        if self.input_mean is not None and self.input_std is not None:
            input_vector = (input_vector - self.input_mean) / self.input_std
        
        # 模型推理
        if self.model_type == "pytorch":
            output_vector = self._predict_pytorch(input_vector)
        elif self.model_type == "tensorflow":
            output_vector = self._predict_tensorflow(input_vector)
        else:
            raise ValueError(f"不支持的模型类型: {self.model_type}")
        
        # 反归一化
        if self.output_mean is not None and self.output_std is not None:
            output_vector = output_vector * self.output_std + self.output_mean
        
        # 构建输出字典
        output_data = {
            feature: float(output_vector[0, i])
            for i, feature in enumerate(self.output_features)
        }
        
        return output_data
    
    def _predict_pytorch(self, input_vector: np.ndarray) -> np.ndarray:
        """PyTorch 模型推理"""
        import torch
        
        with torch.no_grad():
            input_tensor = torch.from_numpy(input_vector).to(self.device)
            output_tensor = self.model(input_tensor)
            output_vector = output_tensor.cpu().numpy()
        
        return output_vector
    
    def _predict_tensorflow(self, input_vector: np.ndarray) -> np.ndarray:
        """TensorFlow 模型推理"""
        output_vector = self.model.predict(input_vector, verbose=0)
        return output_vector
    
    def set_normalization_params(
        self,
        input_mean: np.ndarray,
        input_std: np.ndarray,
        output_mean: np.ndarray,
        output_std: np.ndarray
    ):
        """设置数据归一化参数"""
        self.input_mean = input_mean
        self.input_std = input_std
        self.output_mean = output_mean
        self.output_std = output_std


class ElectrolysisController:
    """电解水平台自动控制器（深度学习版本）"""
    
    def __init__(
        self,
        platform,  # ElectrolysisWaterPlatform 实例
        config: Dict[str, Any]
    ):
        """
        初始化控制器
        
        Args:
            platform: 电解水平台实例
            config: 控制器配置字典
        """
        self.platform = platform
        self.config = config
        
        # 控制模式：0=恒压, 1=恒流
        self.mode = config.get('mode', 0)
        
        # 控制目标和参数
        self.target_voltage = config.get('target_voltage', 3500)  # mV
        self.target_current = config.get('target_current', 1000)  # mA
        self.target_temperature = config.get('target_temperature', 25.0)  # ℃
        self.ki_value = config.get('ki_value', 1.0)
        self.pump_speed = config.get('pump_speed', 50)
        
        # 加载深度学习模型
        model_config = config.get('model', {})
        if model_config:
            self.dl_controller = DeepLearningController(
                model_path=model_config['path'],
                model_type=model_config.get('type', 'pytorch'),
                input_features=model_config.get('input_features'),
                output_features=model_config.get('output_features'),
                device=model_config.get('device', 'cpu')
            )
            
            # 设置归一化参数（如果提供）
            if 'normalization' in model_config:
                norm = model_config['normalization']
                self.dl_controller.set_normalization_params(
                    input_mean=np.array(norm['input_mean']),
                    input_std=np.array(norm['input_std']),
                    output_mean=np.array(norm['output_mean']),
                    output_std=np.array(norm['output_std'])
                )
        else:
            self.dl_controller = None
            print("[Controller] 警告: 未配置深度学习模型，将使用固定控制参数")
        
        # 控制线程
        self.control_thread: Optional[threading.Thread] = None
        self.running = False
        self.control_rate = config.get('control_rate', 1.0)  # Hz
        
        # 数据历史
        self.history_size = config.get('history_size', 100)
        self.voltage_history = deque(maxlen=self.history_size)
        self.current_history = deque(maxlen=self.history_size)
        self.temperature_history = deque(maxlen=self.history_size)
        
        # 回调函数
        self.on_update_callback: Optional[Callable] = None
    
    def start_control(self):
        """启动自动控制"""
        if self.running:
            print("[Controller] 控制器已在运行")
            return
        
        self.running = True
        self.control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="electrolysis_controller"
        )
        self.control_thread.start()
        print(f"[Controller] 自动控制已启动 (模式: {'恒压' if self.mode == 0 else '恒流'})")
    
    def stop_control(self):
        """停止自动控制"""
        if not self.running:
            return
        
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        print("[Controller] 自动控制已停止")
    
    def _control_loop(self):
        """控制循环"""
        sleep_time = 1.0 / self.control_rate
        
        while self.running and self.platform.ser and self.platform.ser.is_open:
            try:
                # 读取当前状态
                current_voltage = self._parse_float(self.platform.voltage)
                current_current = self._parse_float(self.platform.current)
                current_temp = self._parse_float(self.platform.temperature)
                current_ph = self._parse_float(self.platform.ph)
                current_gas_flow = self._parse_float(self.platform.gas_flow)
                current_liquid_flow = self._parse_float(self.platform.liquid_flow)
                
                # 保存历史数据
                self.voltage_history.append(current_voltage)
                self.current_history.append(current_current)
                self.temperature_history.append(current_temp)
                
                # 使用深度学习模型计算控制输出
                if self.dl_controller:
                    # 准备输入数据
                    input_data = {
                        'current': current_current,
                        'voltage': current_voltage,
                        'temperature': current_temp,
                        'ph': current_ph,
                        'gas_flow': current_gas_flow,
                        'liquid_flow': current_liquid_flow,
                        'target_voltage': self.target_voltage,
                        'target_current': self.target_current,
                        'mode': self.mode
                    }
                    
                    # 模型推理
                    output = self.dl_controller.predict(input_data)
                    
                    # 提取控制输出
                    control_voltage = int(output.get('control_voltage', self.target_voltage))
                    control_current = int(output.get('control_current', self.target_current))
                else:
                    # 如果没有模型，使用目标值
                    control_voltage = self.target_voltage
                    control_current = self.target_current
                
                # 应用安全限制
                max_voltage = self.config.get('max_voltage', 5000)
                max_current = self.config.get('max_current', 2000)
                control_voltage = max(0, min(control_voltage, max_voltage))
                control_current = max(0, min(control_current, max_current))
                
                # 构建并发送控制帧
                frame = self.platform.build_tx_frame(
                    mode=self.mode,
                    current_ma=control_current,
                    voltage_mv=control_voltage,
                    temp_c=self.target_temperature,
                    ki=self.ki_value,
                    pump_percent=self.pump_speed
                )
                
                self.platform.ser.write(frame)
                
                # 调用回调函数
                if self.on_update_callback:
                    self.on_update_callback({
                        'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
                        'mode': 'CV' if self.mode == 0 else 'CC',
                        'current_voltage': current_voltage,
                        'current_current': current_current,
                        'current_temperature': current_temp,
                        'control_voltage': control_voltage,
                        'control_current': control_current,
                        'model_output': output if self.dl_controller else None
                    })
                
                # 打印调试信息（可选）
                if hasattr(self, '_debug') and self._debug:
                    print(f"[Controller] V={current_voltage:.0f}mV→{control_voltage:.0f}mV, "
                          f"I={current_current:.0f}mA→{control_current:.0f}mA, "
                          f"T={current_temp:.1f}°C")
                
            except Exception as e:
                print(f"[Controller] 控制循环出错: {e}")
                import traceback
                traceback.print_exc()
            
            time.sleep(sleep_time)
    
    @staticmethod
    def _parse_float(value: str) -> float:
        """安全地将字符串转换为浮点数"""
        try:
            return float(value)
        except (ValueError, TypeError):
            return 0.0
    
    def set_target(self, voltage: Optional[float] = None, 
                   current: Optional[float] = None,
                   temperature: Optional[float] = None):
        """更新控制目标"""
        if voltage is not None:
            self.target_voltage = voltage
        if current is not None:
            self.target_current = current
        if temperature is not None:
            self.target_temperature = temperature
        
        print(f"[Controller] 目标已更新: V={self.target_voltage}mV, "
              f"I={self.target_current}mA, T={self.target_temperature}°C")
    
    def switch_mode(self, mode: int):
        """切换控制模式"""
        if mode not in [0, 1]:
            print("[Controller] 无效的模式，必须是 0(恒压) 或 1(恒流)")
            return
        
        self.mode = mode
        print(f"[Controller] 已切换到 {'恒压' if mode == 0 else '恒流'} 模式")
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        if not self.voltage_history:
            return {}
        
        return {
            'voltage': {
                'current': self.voltage_history[-1],
                'average': sum(self.voltage_history) / len(self.voltage_history),
                'max': max(self.voltage_history),
                'min': min(self.voltage_history)
            },
            'current': {
                'current': self.current_history[-1],
                'average': sum(self.current_history) / len(self.current_history),
                'max': max(self.current_history),
                'min': min(self.current_history)
            },
            'temperature': {
                'current': self.temperature_history[-1],
                'average': sum(self.temperature_history) / len(self.temperature_history),
                'max': max(self.temperature_history),
                'min': min(self.temperature_history)
            }
        }


# ==================== 简单的模型示例 ====================

def create_simple_pytorch_model():
    """创建一个简单的 PyTorch 模型示例"""
    import torch
    import torch.nn as nn
    
    class ElectrolysisControlNet(nn.Module):
        def __init__(self, input_dim=6, hidden_dim=32, output_dim=2):
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
    
    model = ElectrolysisControlNet()
    return model


def create_simple_tensorflow_model():
    """创建一个简单的 TensorFlow 模型示例"""
    import tensorflow as tf
    
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(32, activation='relu', input_shape=(6,)),
        tf.keras.layers.Dense(32, activation='relu'),
        tf.keras.layers.Dense(2)
    ])
    
    model.compile(optimizer='adam', loss='mse')
    return model


# ==================== 使用示例 ====================

if __name__ == "__main__":
    print("="*60)
    print("电解水平台深度学习控制器示例")
    print("="*60)
    
    # 示例 1: 创建并保存模型
    print("\n1. 创建模型示例...")
    
    try:
        import torch
        model = create_simple_pytorch_model()
        torch.save(model, "electrolysis_model.pth")
        print("   ✓ PyTorch 模型已保存: electrolysis_model.pth")
    except ImportError:
        print("   ⚠ PyTorch 未安装，跳过 PyTorch 示例")
    
    try:
        import tensorflow as tf
        model = create_simple_tensorflow_model()
        model.save("electrolysis_model_tf")
        print("   ✓ TensorFlow 模型已保存: electrolysis_model_tf/")
    except ImportError:
        print("   ⚠ TensorFlow 未安装，跳过 TensorFlow 示例")
    
    # 示例 2: 配置示例
    print("\n2. 控制器配置示例:")
    config_example = {
        'mode': 0,
        'target_voltage': 3500,
        'target_current': 1000,
        'target_temperature': 25.0,
        'ki_value': 1.0,
        'pump_speed': 50,
        'control_rate': 1.0,
        
        # 深度学习模型配置
        'model': {
            'path': 'electrolysis_model.pth',  # 或 'electrolysis_model_tf'
            'type': 'pytorch',  # 或 'tensorflow'
            'device': 'cpu',  # 或 'cuda'
            
            # 输入特征（传感器读数）
            'input_features': [
                'current', 'voltage', 'temperature',
                'ph', 'gas_flow', 'liquid_flow'
            ],
            
            # 输出特征（控制命令）
            'output_features': [
                'control_voltage', 'control_current'
            ],
            
            # 数据归一化参数（可选）
            'normalization': {
                'input_mean': [1000, 3500, 25, 7, 50, 30],
                'input_std': [500, 1000, 5, 2, 25, 15],
                'output_mean': [3500, 1000],
                'output_std': [1000, 500]
            }
        }
    }
    
    print("   配置结构:")
    print(f"   - 模型路径: {config_example['model']['path']}")
    print(f"   - 模型类型: {config_example['model']['type']}")
    print(f"   - 输入维度: {len(config_example['model']['input_features'])}")
    print(f"   - 输出维度: {len(config_example['model']['output_features'])}")
    
    # 示例 3: 使用说明
    print("\n3. 使用方法:")
    print("""
   from electrolysis_water_platfrom import ElectrolysisWaterPlatform
   from electrolysis_controller import ElectrolysisController
   
   # 创建平台
   platform = ElectrolysisWaterPlatform(port="COM5")
   
   # 创建控制器（自动加载模型）
   controller = ElectrolysisController(platform, config)
   
   # 启动控制
   controller.start_control()
   
   # 动态调整
   controller.set_target(voltage=4000)
   controller.switch_mode(1)
   
   # 停止控制
   controller.stop_control()
    """)
    
    print("\n" + "="*60)
    print("示例完成！")
    print("="*60)
