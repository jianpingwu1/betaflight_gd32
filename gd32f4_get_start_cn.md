# GD32F4 Betaflight 快速上手指南

本文档介绍如何基于GD32F4构建和编译betaflight固件。本文档基于Ubuntu 22.04 LTS版本进行测试。

## 克隆仓库并安装工具链

克隆仓库将在当前文件夹中创建一个名为betaflight_gd32的文件夹，并下载本地副本。以下命令行将安装必要的先决条件，克隆并设置Betaflight。

### 安装先决条件
```bash
sudo apt -y install build-essential git curl clang-15 python3 python-is-python3
```

### 克隆仓库
```bash
git clone https://github.com/jianpingwu1/betaflight_gd32
cd betaflight_gd32
```

### 切换到GD32F4支持分支
切换到支持GD32F4版本4.5.1的分支：
```bash
git checkout 4.5.1_gd32f4_support
```

### 安装编译工具链
```bash
make arm_sdk_install
```

### 获取配置文件
```bash
make configs
```

### 编译GD32F460飞控固件
```bash
make AOCODARCF460V3_GD HSE_VALUE=8000000
```

## 构建输出

编译成功后，固件文件将在`obj/`目录中生成：
- `betaflight_4.5.1_GD32F460RG_AOCODARCF460V3_GD.hex` - 可用于烧录的固件文件

## 注意事项

- 确保有足够的磁盘空间用于工具链安装
- HSE_VALUE参数指定外部晶振频率（此处为8MHz）
- 构建过程可能需要几分钟，具体取决于系统配置
