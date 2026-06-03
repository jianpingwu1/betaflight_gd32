#
# GD32H7 Make file include
#

LIB_MAIN_DIR    = $(ROOT)/lib/main

CMSIS_DIR      := $(ROOT)/lib/main/CMSIS
STDPERIPH_DIR   = $(ROOT)/lib/main/GD32H7/Firmware/GD32H7xx_standard_peripheral
USB_LIB_DIR    := $(ROOT)/lib/main/GD32H7/Firmware/GD32H7xx_usbhs_library

STDPERIPH_SRC   = \
            gd32h7xx_adc.c \
            gd32h7xx_dac.c \
            gd32h7xx_dma.c \
            gd32h7xx_exti.c \
            gd32h7xx_fmc.c \
            gd32h7xx_dbg.c \
            gd32h7xx_dci.c \
            gd32h7xx_fwdgt.c \
            gd32h7xx_gpio.c \
            gd32h7xx_i2c.c \
            gd32h7xx_ipa.c \
            gd32h7xx_misc.c \
            gd32h7xx_ospi.c \
            gd32h7xx_ospim.c \
            gd32h7xx_pmu.c \
            gd32h7xx_rcu.c \
            gd32h7xx_rtc.c \
            gd32h7xx_rtdec.c \
            gd32h7xx_sdio.c \
            gd32h7xx_spi.c \
            gd32h7xx_syscfg.c \
            gd32h7xx_timer.c \
            gd32h7xx_trigsel.c \
            gd32h7xx_tli.c \
            gd32h7xx_trng.c \
            gd32h7xx_usart.c \
            gd32h7xx_wwdgt.c

VPATH       := $(VPATH):$(STDPERIPH_DIR)/Source

DEVICE_FLAGS = -DGD32H7XX

# USB Device
USBDEVICE_DIR = $(USB_LIB_DIR)/device

USBDCORE_DIR = $(USBDEVICE_DIR)/core
USBCORE_SRC  = \
                 $(USBDCORE_DIR)/Source/usbd_core.c \
                 $(USBDCORE_DIR)/Source/usbd_enum.c \
                 $(USBDCORE_DIR)/Source/usbd_transc.c

USBCDC_DIR   = $(USBDEVICE_DIR)/class/cdc
USBCDC_SRC   = \
               $(USBCDC_DIR)/Source/cdc_acm_core.c

USBMSC_DIR   = $(USBDEVICE_DIR)/class/msc
USBMSC_SRC   = \
               $(USBMSC_DIR)/Source/usbd_msc_bbb.c \
               $(USBMSC_DIR)/Source/usbd_msc_core.c \
               $(USBMSC_DIR)/Source/usbd_msc_scsi.c

USBHID_DIR   = $(USBDEVICE_DIR)/class/hid
USBHID_SRC   = \
               $(USBHID_DIR)/Source/standard_hid_core.c


USBDRV_DIR   = $(USB_LIB_DIR)/driver
USBDRV_SRC   = \
               $(USB_LIB_DIR)/driver/Source/drv_usb_core.c \
               $(USB_LIB_DIR)/driver/Source/drv_usb_dev.c \
               $(USB_LIB_DIR)/driver/Source/drv_usbd_int.c


USBSTD_DIR   = $(USB_LIB_DIR)/ustd


DEVICE_STDPERIPH_SRC := \
            $(STDPERIPH_SRC) \
            $(USBCORE_SRC) \
            $(USBCDC_SRC) \
            $(USBHID_SRC) \
            $(USBMSC_SRC) \
            $(USBDRV_SRC)

VPATH        := $(VPATH):$(CMSIS_DIR)/Core/Include:$(LIB_MAIN_DIR)/GD32H7/Firmware/CMSIS/GD/GD32H7xx/Source

CMSIS_SRC    :=

INCLUDE_DIRS    := \
            $(INCLUDE_DIRS) \
            $(ROOT)/src/main/drivers/GD32 \
            $(ROOT)/src/main/startup/GD32 \
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
            $(LIB_MAIN_DIR)/GD32H7/Firmware/CMSIS/GD/GD32H7xx/Include \
            $(ROOT)/src/main/drivers/GD32/usb_h7

DEVICE_FLAGS += -DUSE_STDPERIPH_DRIVER


#Flags
ARCH_FLAGS      = -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16

ifeq ($(TARGET_MCU),GD32H757VI)
DEVICE_FLAGS       += -DGD32H757 -DGD32H7XXV
DEFAULT_LD_SCRIPT  = $(LINKER_DIR)/gd32h757xi_flash.ld
STARTUP_SRC        = startup/GD32/startup_gd32h757.S
MCU_FLASH_SIZE     := 2048

else
$(error Unknown MCU for H7 target)
endif

ifeq ($(LD_SCRIPT),)
LD_SCRIPT = $(DEFAULT_LD_SCRIPT)
endif

ifneq ($(FIRMWARE_SIZE),)
DEVICE_FLAGS   += -DFIRMWARE_SIZE=$(FIRMWARE_SIZE)
endif

DEVICE_FLAGS    += -DHXTAL_VALUE=$(HSE_VALUE)

DSP_LIB := $(LIB_MAIN_DIR)/CMSIS/DSP
DEVICE_FLAGS += -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE -DARM_MATH_CM7
DEVICE_FLAGS += -DUSE_GDBSP_DRIVER -DUSE_USBHS0
DEVICE_FLAGS += -DUSE_USB_FS -DUSE_IRC48M
# DEVICE_FLAGS += -DUSE_USB_HS
# DEVICE_FLAGS += -DVECT_TAB_SRAM

MCU_COMMON_SRC = \
            drivers/accgyro/accgyro_mpu.c \
            drivers/dshot_bitbang_decode.c \
            drivers/inverter.c \
            drivers/pwm_output_dshot_shared.c \
            drivers/GD32/adc_gd32h7xx.c \
            drivers/GD32/audio_gd32h7xx.c \
            drivers/GD32/bus_i2c_gd32h7xx.c \
            drivers/GD32/bus_spi_gd32h7xx.c \
            drivers/GD32/bus_ospi_gd32h7xx.c \
            drivers/GD32/camera_control_gd32.c \
            drivers/GD32/debug.c \
            drivers/GD32/dma_gd32h7xx.c \
            drivers/GD32/dma_reqmap_mcu.c \
            drivers/GD32/dshot_bitbang.c \
            drivers/GD32/dshot_bitbang_stdperiph.c \
            drivers/GD32/exti_gd32.c \
            drivers/GD32/io_gd32.c \
            drivers/GD32/light_ws2811strip_stdperiph.c \
            drivers/GD32/memprot_gd32.c \
            drivers/GD32/memprot_gd32h7xx.c \
            drivers/GD32/persistent_gd32.c \
            drivers/GD32/rcu_gd32.c \
            drivers/GD32/sdio_gd32h7xx.c \
            drivers/GD32/serial_uart_stdperiph.c \
            drivers/GD32/serial_uart_gd32h7xx.c \
            drivers/GD32/pwm_output_gd32.c \
            drivers/GD32/pwm_output_dshot.c \
            drivers/GD32/timer_stdperiph.c \
            drivers/GD32/timer_gd32h7xx.c \
            drivers/GD32/transponder_ir_io_stdperiph.c \
            drivers/GD32/usb_h7/usbd_msc_desc.c \
            drivers/adc.c \
            drivers/bus_spi_config.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart_pinconfig.c \
            drivers/GD32/system_gd32h7xx.c \
            startup/GD32/system_gd32h7xx.c

VCP_SRC = \
            drivers/GD32/usb_h7/gd32h7xx_it.c \
            drivers/GD32/usb_h7/usb_bsp.c \
            drivers/GD32/usb_h7/usbd_desc.c \
            drivers/GD32/usb_h7/usb_cdc_hid.c \
            drivers/GD32/usbd_cdc_vcp.c \
            drivers/GD32/serial_usb_vcp.c \
            drivers/usb_io.c

MSC_SRC = \
            drivers/usb_msc_common.c \
            drivers/GD32/usb_msc_h7xx.c \
            msc/usbd_storage.c \
            msc/usbd_storage_emfat.c \
            msc/emfat.c \
            msc/emfat_file.c \
            msc/usbd_storage_sd_spi.c \
            msc/usbd_storage_sdio.c

SIZE_OPTIMISED_SRC += \
            drivers/GD32/serial_usb_vcp.c \
            drivers/inverter.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/bus_spi_config.c

