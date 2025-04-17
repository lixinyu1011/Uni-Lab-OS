"""
日志适配器模块

用于将各种框架的日志（如Uvicorn、FastAPI等）统一适配到ilabos的日志系统
"""

import logging

from unilabos.utils.log import debug, info, warning, error, critical


class UvicornLogAdapter:
    """Uvicorn日志适配器，将Uvicorn的日志重定向到我们的日志系统"""

    @staticmethod
    def configure():
        """配置Uvicorn的日志系统，使用我们自定义的日志格式"""
        # 获取uvicorn相关的日志记录器
        uvicorn_loggers = [
            logging.getLogger("uvicorn"),
            logging.getLogger("uvicorn.access"),
            logging.getLogger("uvicorn.error"),
            logging.getLogger("fastapi"),
        ]

        # 清除现有处理器
        for logger_instance in uvicorn_loggers:
            for handler in logger_instance.handlers[:]:
                logger_instance.removeHandler(handler)

        # 添加自定义处理器
        adapter_handler = UvicornToIlabosHandler()

        # 为所有uvicorn日志记录器添加处理器
        for logger_instance in uvicorn_loggers:
            logger_instance.addHandler(adapter_handler)
            # 设置日志级别
            logger_instance.setLevel(logging.INFO)
            # 禁止传播到根日志记录器，避免重复输出
            logger_instance.propagate = False


class UvicornToIlabosHandler(logging.Handler):
    """将Uvicorn日志处理为ilabos日志格式的处理器"""

    def __init__(self):
        super().__init__()
        self.level_map = {
            logging.DEBUG: debug,
            logging.INFO: info,
            logging.WARNING: warning,
            logging.ERROR: error,
            logging.CRITICAL: critical,
        }

    def emit(self, record):
        """发送日志记录到ilabos日志系统"""
        try:
            msg = self.format(record)
            log_func = self.level_map.get(record.levelno, info)
            # 根据日志源添加前缀
            if record.name.startswith("uvicorn"):
                prefix = "[Uvicorn] "
                if record.name == "uvicorn.access":
                    prefix = "[Uvicorn.HTTP] "
                msg = f"{prefix}{msg}"
            elif record.name.startswith("fastapi"):
                msg = f"[FastAPI] {msg}"
            else:
                msg = f"{record.name} {msg}"
            log_func(msg, stack_level=5)
        except Exception:
            self.handleError(record)


def setup_fastapi_logging():
    """设置FastAPI/Uvicorn的日志系统"""
    # 配置Uvicorn的日志
    UvicornLogAdapter.configure()

    # 返回适合uvicorn.run()的日志配置
    return {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "default": {
                "()": "uvicorn.logging.DefaultFormatter",
                "fmt": "%(message)s",
                "use_colors": True,
            },
        },
        "handlers": {
            "default": {
                "formatter": "default",
                "class": "unilabos.utils.fastapi.log_adapter.UvicornToIlabosHandler",
            }
        },
        "loggers": {
            "uvicorn": {"handlers": ["default"], "level": "INFO"},
            "uvicorn.error": {"handlers": ["default"], "level": "INFO"},
            "uvicorn.access": {"handlers": ["default"], "level": "INFO"},
            "fastapi": {"handlers": ["default"], "level": "INFO"},
        },
    }
