# GD32F4 Betaflight Getting Started Guide

This document explains how to build and compile betaflight firmware based on GD32F4. This document is tested and based on Ubuntu 22.04 LTS release.

## Clone the Repository and Install Toolchain

Cloning the repository will create a folder named betaflight_gd32 in your current folder and download a copy for local use. The following command-lines will install necessary prerequisites, clone and setup Betaflight.

### Install Prerequisites
```bash
sudo apt -y install build-essential git curl clang-15 python3 python-is-python3
```

### Clone Repository
```bash
git clone https://github.com/jianpingwu1/betaflight_gd32
cd betaflight_gd32
```

### Switch to GD32F4 Support Branch
Switch to the branch that supports GD32F4 for version 4.5.1:
```bash
git checkout 4.5.1_gd32f4_support
```

### Install Compilation Toolchain
```bash
make arm_sdk_install
```

### Get Configuration Files
```bash
make configs
```

### Compile GD32F460 Flight Controller Firmware
```bash
make AOCODARCF460V3_GD HSE_VALUE=8000000
```

## Build Output

After successful compilation, the firmware files will be generated in the `obj/` directory:
- `betaflight_4.5.1_GD32F460RG_AOCODARCF460V3_GD.hex` - The firmware file ready for flashing

## Notes

- Make sure you have sufficient disk space for the toolchain installation
- The HSE_VALUE parameter specifies the external crystal frequency (8MHz in this case)
- The build process may take several minutes depending on your system specifications
