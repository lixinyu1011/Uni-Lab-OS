from unilabos.utils.log import logger

# 确保日志配置在导入utils包时自动应用
# 这样任何导入utils包或其子模块的代码都会自动配置好日志

# 导出logger，使其可以通过from unilabos.utils import logger直接导入
__all__ = ['logger']
