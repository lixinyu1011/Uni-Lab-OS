# **Uni-Lab 安装**

## 快速开始

1. **配置 Conda 环境**

Uni-Lab-OS 建议使用 `mamba` 管理环境。创建新的环境：

```shell
mamba create -n unilab uni-lab::unilabos -c robostack-staging -c conda-forge
```

2. **安装开发版 Uni-Lab-OS**

```shell
# 配置好conda环境后，克隆仓库
git clone https://github.com/dptech-corp/Uni-Lab-OS.git
cd Uni-Lab-OS

# 安装 Uni-Lab-OS
pip install .
```

3. **启动 Uni-Lab 系统**

请参见{doc}`启动样例 <../boot_examples/index>`或{doc}`启动指南 <launch>`了解详细的启动方法。
