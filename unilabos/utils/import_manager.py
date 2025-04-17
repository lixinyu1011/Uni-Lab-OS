"""
导入管理器

该模块提供了一个动态导入和管理模块的系统，避免误删未使用的导入。
"""

import builtins
import importlib
import inspect
import traceback
from typing import Dict, List, Any, Optional, Callable, Type


__all__ = [
    "ImportManager",
    "default_manager",
    "load_module",
    "get_class",
    "get_module",
    "init_from_list",
]

from unilabos.utils import logger


class ImportManager:
    """导入管理器类，用于动态加载和管理模块"""

    def __init__(self, module_list: Optional[List[str]] = None):
        """
        初始化导入管理器

        Args:
            module_list: 要预加载的模块路径列表
        """
        self._modules: Dict[str, Any] = {}
        self._classes: Dict[str, Type] = {}
        self._functions: Dict[str, Callable] = {}

        if module_list:
            for module_path in module_list:
                self.load_module(module_path)

    def load_module(self, module_path: str) -> Any:
        """
        加载指定路径的模块

        Args:
            module_path: 模块路径

        Returns:
            加载的模块对象

        Raises:
            ImportError: 如果模块导入失败
        """
        try:
            if module_path in self._modules:
                return self._modules[module_path]

            module = importlib.import_module(module_path)
            self._modules[module_path] = module

            # 索引模块中的类和函数
            for name, obj in inspect.getmembers(module):
                if inspect.isclass(obj):
                    full_name = f"{module_path}.{name}"
                    self._classes[name] = obj
                    self._classes[full_name] = obj
                elif inspect.isfunction(obj):
                    full_name = f"{module_path}.{name}"
                    self._functions[name] = obj
                    self._functions[full_name] = obj

            return module
        except Exception as e:
            logger.error(f"导入模块 '{module_path}' 时发生错误：{str(e)}")
            logger.warning(traceback.format_exc())
            raise ImportError(f"无法导入模块 {module_path}: {str(e)}")

    def get_module(self, module_path: str) -> Any:
        """
        获取已加载的模块

        Args:
            module_path: 模块路径

        Returns:
            模块对象

        Raises:
            KeyError: 如果模块未加载
        """
        if module_path not in self._modules:
            return self.load_module(module_path)
        return self._modules[module_path]

    def get_class(self, class_name: str) -> Type:
        """
        获取类对象

        Args:
            class_name: 类名或完整类路径

        Returns:
            类对象

        Raises:
            KeyError: 如果找不到类
        """
        if class_name in self._classes:
            return self._classes[class_name]

        # 尝试动态导入
        if ":" in class_name:
            module_path, cls_name = class_name.rsplit(":", 1)
            # 如果cls_name是builtins中的关键字，则返回对应类
            if cls_name in builtins.__dict__:
                return builtins.__dict__[cls_name]
            module = self.load_module(module_path)
            if hasattr(module, cls_name):
                cls = getattr(module, cls_name)
                self._classes[class_name] = cls
                self._classes[cls_name] = cls
                return cls

        raise KeyError(f"找不到类: {class_name}")

    def list_modules(self) -> List[str]:
        """列出所有已加载的模块路径"""
        return list(self._modules.keys())

    def list_classes(self) -> List[str]:
        """列出所有已索引的类名"""
        return list(self._classes.keys())

    def list_functions(self) -> List[str]:
        """列出所有已索引的函数名"""
        return list(self._functions.keys())

    def search_class(self, class_name: str, search_lower=False) -> Optional[Type]:
        """
        在所有已加载的模块中搜索特定类名

        Args:
            class_name: 要搜索的类名
            search_lower: 以小写搜索

        Returns:
            找到的类对象，如果未找到则返回None
        """
        # 首先在已索引的类中查找
        if class_name in self._classes:
            return self._classes[class_name]

        if search_lower:
            classes = {name.lower(): obj for name, obj in self._classes.items()}
            if class_name in classes:
                return classes[class_name]

        # 遍历所有已加载的模块进行搜索
        for module_path, module in self._modules.items():
            for name, obj in inspect.getmembers(module):
                if inspect.isclass(obj) and ((name.lower() == class_name.lower()) if search_lower else (name == class_name)):
                    # 将找到的类添加到索引中
                    self._classes[name] = obj
                    self._classes[f"{module_path}:{name}"] = obj
                    return obj

        return None


# 全局实例，便于直接使用
default_manager = ImportManager()


def load_module(module_path: str) -> Any:
    """加载模块的便捷函数"""
    return default_manager.load_module(module_path)


def get_class(class_name: str) -> Type:
    """获取类的便捷函数"""
    return default_manager.get_class(class_name)


def get_module(module_path: str) -> Any:
    """获取模块的便捷函数"""
    return default_manager.get_module(module_path)


def init_from_list(module_list: List[str]) -> None:
    """从模块列表初始化默认管理器"""
    global default_manager
    default_manager = ImportManager(module_list)
