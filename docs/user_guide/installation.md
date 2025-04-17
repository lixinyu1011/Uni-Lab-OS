# **Uni-Lab 安装**

请先 `git clone` 本仓库，随后按照以下步骤安装项目：

`Uni-Lab` 建议您采用 `mamba` 管理环境。若需从头建立 `Uni-Lab` 的运行依赖环境，请执行

```shell
mamba env create -f unilabos-<YOUR_OS>.yaml
mamba activate ilab
```

其中 `YOUR_OS` 是您的操作系统，可选值 `win64`, `linux-64`, `osx-64`, `osx-arm64`

若需将依赖安装进当前环境，请执行

```shell
conda env update --file unilabos-<YOUR_OS>.yml
```

随后，可在本仓库安装 `unilabos` 的开发版：

```shell
pip install .
```
