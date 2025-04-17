def singleton(cls):
    """
    单例装饰器
    确保被装饰的类只有一个实例
    """
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance

