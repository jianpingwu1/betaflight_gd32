#
# GD32F4 Make file include
#

LIB_MAIN_DIR = $(ROOT)/lib/main

CMSIS_DIR      := $(ROOT)/lib/main/CMSIS
STDPERIPH_DIR   = $(ROOT)/lib/main/GD32F4/Drivers/GD32F4xx_standard_peripheral
USB_LIB_DIR    := $(ROOT)/lib/main/GD32F4/Middlewares/GD32F4xx_usb_library

STDPERIPH_SRC   = \
            gd32f4xx_adc.c \
            gd32f4xx_dac.c \
            gd32f4xx_dci.c \
            gd32f4xx_dma.c \
            gd32f4xx_exti.c \
            gd32f4xx_fmc.c \
            gd32f4xx_fwdgt.c \
            gd32f4xx_gpio.c \
            gd32f4xx_i2c.c \
            gd32f4xx_ipa.c \
            gd32f4xx_misc.c\
            gd32f4xx_pmu.c \
            gd32f4xx_rcu.c \
            gd32f4xx_rtc.c \
            gd32f4xx_sdio.c \
            gd32f4xx_spi.c \
            gd32f4xx_syscfg.c \
            gd32f4xx_timer.c \
            gd32f4xx_tli.c \
            gd32f4xx_trng.c \
            gd32f4xx_usart.c \
            gd32f4xx_wwdgt.c

VPATH       := $(VPATH):$(STDPERIPH_DIR)/Source

DEVICE_FLAGS = -DGD32F4XX

ifneq ($(TARGET_MCU),$(filter $(TARGET_MCU),GDF450 GD32F470))
STDPERIPH_SRC += gd32f4xx_exmc.c
endif


USBDEVICE_DIR = $(USB_LIB_DIR)/device

USBDCORE_DIR = $(USBDEVICE_DIR)/core
USBCORE_SRC = \
                $(USBDCORE_DIR)/Source/usbd_core.c \
                $(USBDCORE_DIR)/Source/usbd_enum.c \
                $(USBDCORE_DIR)/Source/usbd_transc.c

USBCDC_DIR  = $(USBDEVICE_DIR)/class/cdc
USBCDC_SRC  = \
              $(USBCDC_DIR)/Source/cdc_acm_core.c

USBMSC_DIR  = $(USBDEVICE_DIR)/class/msc
USBMSC_SRC  = \
              $(USBMSC_DIR)/Source/usbd_msc_bbb.c \
              $(USBMSC_DIR)/Source/usbd_msc_core.c \
              $(USBMSC_DIR)/Source/usbd_msc_scsi.c

USBHID_DIR  = $(USBDEVICE_DIR)/class/hid
USBHID_SRC  = \
              $(USBHID_DIR)/Source/standard_hid_core.c


USBDRV_DIR  = $(USB_LIB_DIR)/driver
USBDRV_SRC  = \
              $(USB_LIB_DIR)/driver/Source/drv_usb_core.c \
              $(USB_LIB_DIR)/driver/Source/drv_usb_dev.c \
              $(USB_LIB_DIR)/driver/Source/drv_usbd_int.c



USBSTD_DIR  = $(USB_LIB_DIR)/ustd


DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBOTG_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC) \
            $(USBHID_SRC) \
            $(USBMSC_SRC) \
            $(USBDRV_SRC)

#CMSIS
VPATH        := $(VPATH):$(CMSIS_DIR)/Core/Include:$(LIB_MAIN_DIR)/GD32F4/Drivers/CMSIS/GD/GD32F4xx/Source


CMSIS_SRC       := \
            gd32f4xx_gpio.c \
            gd32f4xx_rcu.c

INCLUDE_DIRS    := \
            $(INCLUDE_DIRS) \
            $(ROOT)/src/main/drivers/GD32 \
            $(ROOT)/src/main/startup/GD32 \
            $(TARGET_PLATFORM_DIR) \
            $(TARGET_PLATFORM_DIR)/startup \
            $(STDPERIPH_DIR)/Include \
            $(USBDEVICE_DIR)/class/msc/Include \
            $(USBHID_DIR)/Include \
            $(USBCDC_DIR)/Include \
            $(USBMSC_DIR)/Include \
            $(USBDCORE_DIR)/Include \
            $(USBDRV_DIR)/Include \
            $(USBSTD_DIR)/common \
            $(USBSTD_DIR)/class/cdc \
            $(USBSTD_DIR)/class/msc \
            $(USBSTD_DIR)/class/hid \
            $(CMSIS_DIR)/Core/Include \
            $(LIB_MAIN_DIR)/GD32F4/Drivers/CMSIS/GD/GD32F4xx/Include \
            $(ROOT)/src/main/drivers/GD32/vcpf4 \
            $(TARGET_PLATFORM_DIR)/vcpf4


DEVICE_FLAGS += -DUSE_STDPERIPH_DRIVER


#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant

ifeq ($(TARGET_MCU),GD32F407xx)
DEVICE_FLAGS    += -DGD32F407
LD_SCRIPT       = $(LINKER_DIR)/gd32_flash_f407.ld
STARTUP_SRC     = GD32/startup_gd32f407_427.s
MCU_FLASH_SIZE  := 1024

else ifeq ($(TARGET_MCU),GD32F425xx)
DEVICE_FLAGS    += -DGD32F425
LD_SCRIPT       = $(LINKER_DIR)/gd32f425rg_flash.ld
STARTUP_SRC     = GD32/startup_gd32f405_425.S
MCU_FLASH_SIZE  := 1024

else ifeq ($(TARGET_MCU),GD32F460xg)
DEVICE_FLAGS    += -DGD32F460
LD_SCRIPT       = $(LINKER_DIR)/gd32f460xg_flash.ld
STARTUP_SRC     = GD32/startup_gd32f460.S
MCU_FLASH_SIZE  := 1024

else
$(error Unknown MCU for F4 target)
endif
DEVICE_FLAGS    += -DHXTAL_VALUE=$(HSE_VALUE)

MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/pwm_output_dshot_shared.c \
            drivers/GD32/adc_gd32.c \
            drivers/GD32/audio_gd32.c \
            drivers/GD32/bus_i2c_gd32.c \
            drivers/GD32/bus_spi_gd32.c \
            drivers/GD32/camera_control_gd32.c \
            drivers/GD32/debug.c \
            drivers/GD32/dma_gd32.c \
            drivers/GD32/dma_reqmap_mcu.c \
            drivers/GD32/dshot_bitbang.c \
            drivers/GD32/dshot_bitbang_stdperiph.c \
            drivers/GD32/exti_gd32.c \
            drivers/GD32/io_gd32.c \
            drivers/GD32/light_ws2811strip_stdperiph.c \
            drivers/GD32/persistent_gd32.c \
            drivers/GD32/rcu_gd32.c \
            drivers/GD32/sdio_gdf4xx.c \
            drivers/GD32/serial_uart_stdperiph.c \
            drivers/GD32/serial_uart_gd32f4xx.c \
            drivers/GD32/system_gd32f4xx.c \
            drivers/GD32/pwm_output_hw.c \
            drivers/GD32/pwm_output_dshot.c \
            drivers/GD32/timer_stdperiph.c \
            drivers/GD32/timer_gd32f4xx.c \
            drivers/GD32/transponder_ir_io_stdperiph.c \
            drivers/GD32/usbd_msc_desc.c \
            drivers/adc.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart_pinconfig.c \
            startup/GD32/system_gd32f4xx.c

VCP_SRC = \
            drivers/GD32/vcpf4/gd32f4xx_it.c \
            drivers/GD32/vcpf4/usb_bsp.c \
            drivers/GD32/vcpf4/usbd_desc.c \
            drivers/GD32/vcpf4/usbd_cdc_vcp.c \
            drivers/GD32/vcpf4/usb_cdc_hid.c \
            drivers/GD32/serial_usb_vcp.c \
            drivers/usb_io.c


MSC_SRC = \
            drivers/usb_msc_common.c \
            drivers/GD32/usb_msc_f4xx.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM4
DEVICE_FLAGS += -DUSE_GDBSP_DRIVER -DUSE_USB_FS
DEVICE_FLAGS += -DVECT_TAB_SRAM
