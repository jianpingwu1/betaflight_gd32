
CONFIGS_REPO_URL ?= https://github.com/jianpingwu1/betaflight_config_gd32
# Optional: branch to checkout for the configs repo
# Leave empty to keep previous behavior
CONFIGS_REPO_BRANCH ?= gd32f4_config_for_BF451

BASE_CONFIGS      = $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(CONFIG_DIR)/configs/*/config.h)))))

ifneq ($(filter-out %_install test% %_clean clean% %-print %.hex %.h hex checks help configs $(BASE_TARGETS) $(BASE_CONFIGS),$(MAKECMDGOALS)),)
ifeq ($(wildcard $(CONFIG_DIR)/configs/),)
$(error `$(CONFIG_DIR)` not found. Have you hydrated configuration using: 'make configs'?)
endif
endif

ifneq ($(CONFIG),)

ifneq ($(TARGET),)
$(error TARGET or CONFIG should be specified. Not both.)
endif

CONFIG_FILE      = $(CONFIG_DIR)/configs/$(CONFIG)/config.h
INCLUDE_DIRS    += $(CONFIG_DIR)/configs/$(CONFIG)

ifneq ($(wildcard $(CONFIG_FILE)),)

CONFIG_REVISION := norevision
ifeq ($(shell git -C $(CONFIG_DIR) diff --shortstat),)
CONFIG_REVISION := $(shell git -C $(CONFIG_DIR) log -1 --format="%h")
CONFIG_REVISION_DEFINE := -D'__CONFIG_REVISION__="$(CONFIG_REVISION)"'
endif

TARGET        := $(shell grep " FC_TARGET_MCU" $(CONFIG_FILE) | awk '{print $$3}' )
HSE_VALUE_MHZ := $(shell grep " SYSTEM_HSE_MHZ" $(CONFIG_FILE) | awk '{print $$3}' )
ifneq ($(HSE_VALUE_MHZ),)
HSE_VALUE     := $(shell echo $$(( $(HSE_VALUE_MHZ) * 1000000 )) )
endif

GYRO_DEFINE   := $(shell grep " USE_GYRO_" $(CONFIG_FILE) | awk '{print $$2}' )

ifeq ($(TARGET),)
$(error No TARGET identified. Is the $(CONFIG_FILE) valid for $(CONFIG)?)
endif

EXST_ADJUST_VMA := $(shell grep " FC_VMA_ADDRESS" $(CONFIG_FILE) | awk '{print $$3}' )
ifneq ($(EXST_ADJUST_VMA),)
EXST = yes
endif

else #exists
$(error `$(CONFIG_FILE)` not found. Have you hydrated configuration using: 'make configs'?)
endif #config_file exists
endif #config

.PHONY: configs
configs:
ifeq ($(wildcard $(CONFIG_DIR)),)
	@echo "Hydrating clone for configs: $(CONFIG_DIR)"
ifneq ($(strip $(CONFIGS_REPO_BRANCH)),)
	$(V0) git clone -b $(CONFIGS_REPO_BRANCH) --single-branch $(CONFIGS_REPO_URL) $(CONFIG_DIR)
else
	$(V0) git clone $(CONFIGS_REPO_URL) $(CONFIG_DIR)
endif
else
ifneq ($(strip $(CONFIGS_REPO_BRANCH)),)
	$(V0) git -C $(CONFIG_DIR) fetch origin $(CONFIGS_REPO_BRANCH)
	$(V0) git -C $(CONFIG_DIR) checkout $(CONFIGS_REPO_BRANCH)
	$(V0) git -C $(CONFIG_DIR) pull --ff-only origin $(CONFIGS_REPO_BRANCH)
else
	$(V0) git -C $(CONFIG_DIR) pull origin
endif
endif

$(BASE_CONFIGS):
	@echo "Building target config $@"
	$(V0) $(MAKE) -j hex CONFIG=$@
	@echo "Building target config $@ succeeded."

## <CONFIG>_rev    : build configured target and add revision to filename
$(addsuffix _rev,$(BASE_CONFIGS)):
	$(V0) $(MAKE) -j hex CONFIG=$(subst _rev,,$@) REV=yes
