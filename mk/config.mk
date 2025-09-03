
CONFIGS_REPO_URL ?= https://github.com/jianpingwu1/betaflight_config_gd32
# Optional: branch to checkout for the configs repo
# Leave empty to keep previous behavior
CONFIGS_REPO_BRANCH ?= gd32f4-config_for_master
# handle only this directory as config submodule
CONFIGS_SUBMODULE_DIR = src/config
BASE_CONFIGS      = $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(CONFIG_DIR)/configs/*/config.h)))))

ifneq ($(filter-out %_sdk %_install test% %_clean clean% %-print %.hex %.h hex checks help configs $(BASE_TARGETS) $(BASE_CONFIGS),$(MAKECMDGOALS)),)
ifeq ($(wildcard $(CONFIG_DIR)/configs/),)
$(error `$(CONFIG_DIR)` not found. Have you hydrated configuration using: 'make configs'?)
endif
endif

ifneq ($(CONFIG),)

ifneq ($(TARGET),)
$(error TARGET or CONFIG should be specified. Not both.)
endif

CONFIG_HEADER_FILE  = $(CONFIG_DIR)/configs/$(CONFIG)/config.h
CONFIG_SOURCE_FILE  = $(CONFIG_DIR)/configs/$(CONFIG)/config.c
INCLUDE_DIRS       += $(CONFIG_DIR)/configs/$(CONFIG)

ifneq ($(wildcard $(CONFIG_HEADER_FILE)),)

CONFIG_SRC :=
ifneq ($(wildcard $(CONFIG_SOURCE_FILE)),)
CONFIG_SRC += $(CONFIG_SOURCE_FILE)
TARGET_FLAGS += -DUSE_CONFIG_SOURCE
endif

CONFIG_REVISION := norevision
ifeq ($(shell git -C $(CONFIG_DIR) diff --shortstat),)
CONFIG_REVISION := $(shell git -C $(CONFIG_DIR) log -1 --format="%h")
CONFIG_REVISION_DEFINE := -D'__CONFIG_REVISION__="$(CONFIG_REVISION)"'
endif

HSE_VALUE_MHZ := $(shell sed -E -n "/^[[:space:]]*\#[[:space:]]*define[[:space:]]+SYSTEM_HSE_MHZ[[:space:]]+([0-9]+).*/s//\1/p" $(CONFIG_HEADER_FILE))
ifneq ($(HSE_VALUE_MHZ),)
HSE_VALUE     := $(shell echo $$(( $(HSE_VALUE_MHZ) * 1000000 )) )
endif

TARGET        := $(shell sed -E -n "/^[[:space:]]*\#[[:space:]]*define[[:space:]]+FC_TARGET_MCU[[:space:]]+([[:alnum:]_]+).*/s//\1/p" $(CONFIG_HEADER_FILE))
ifeq ($(TARGET),)
$(error No TARGET identified. Is the $(CONFIG_HEADER_FILE) valid for $(CONFIG)?)
endif

EXST_ADJUST_VMA := $(shell sed -E -n "/^[[:space:]]*\#[[:space:]]*define[[:space:]]+FC_VMA_ADDRESS[[:space:]]+((0[xX])?[[:xdigit:]]+).*/s//\1/p" $(CONFIG_HEADER_FILE))
ifneq ($(EXST_ADJUST_VMA),)
EXST = yes
endif

else #exists
$(error `$(CONFIG_HEADER_FILE)` not found. Have you hydrated configuration using: 'make configs'?)
endif #CONFIG_HEADER_FILE exists
endif #config

.PHONY: configs
configs:
ifeq ($(shell realpath $(CONFIG_DIR)),$(shell realpath $(CONFIGS_SUBMODULE_DIR)))
	@echo "Updating config submodule: $(CONFIGS_SUBMODULE_DIR)"
	$(V1) git submodule update --init $(if $(strip $(CONFIGS_REPO_BRANCH)),--remote,) -- $(CONFIGS_SUBMODULE_DIR) || { echo "Config submodule update failed. Please check your git configuration."; exit 1; }
ifneq ($(strip $(CONFIGS_REPO_BRANCH)),)
	$(V1) echo "Switching submodule to branch '$(CONFIGS_REPO_BRANCH)'"
	$(V1) git -C $(CONFIGS_SUBMODULE_DIR) fetch origin $(CONFIGS_REPO_BRANCH) || { echo "Submodule fetch failed."; exit 1; }
	$(V1) git -C $(CONFIGS_SUBMODULE_DIR) checkout $(CONFIGS_REPO_BRANCH) || { echo "Submodule checkout failed."; exit 1; }
	$(V1) git -C $(CONFIGS_SUBMODULE_DIR) pull --ff-only origin $(CONFIGS_REPO_BRANCH) || { echo "Submodule pull failed."; exit 1; }
endif
	@echo "Submodule update succeeded."
else
ifeq ($(wildcard $(CONFIG_DIR)),)
	@echo "Hydrating clone for configs: $(CONFIG_DIR)"
ifneq ($(strip $(CONFIGS_REPO_BRANCH)),)
	$(V1) git clone -b $(CONFIGS_REPO_BRANCH) --single-branch $(CONFIGS_REPO_URL) $(CONFIG_DIR) || { echo "Clone with branch failed."; exit 1; }
else
	$(V1) git clone $(CONFIGS_REPO_URL) $(CONFIG_DIR) || { echo "Clone failed."; exit 1; }
endif
else
ifneq ($(strip $(CONFIGS_REPO_BRANCH)),)
	$(V1) git -C $(CONFIG_DIR) fetch origin $(CONFIGS_REPO_BRANCH) || { echo "Fetch branch failed."; exit 1; }
	$(V1) git -C $(CONFIG_DIR) checkout $(CONFIGS_REPO_BRANCH) || { echo "Checkout branch failed."; exit 1; }
	$(V1) git -C $(CONFIG_DIR) pull --ff-only origin $(CONFIGS_REPO_BRANCH) || { echo "Pull branch failed."; exit 1; }
else
	$(V1) git -C $(CONFIG_DIR) pull origin || { echo "Pull failed."; exit 1; }
endif
endif
endif

$(BASE_CONFIGS):
	@echo "Building target config $@"
	$(V0) $(MAKE) fwo CONFIG=$@
	@echo "Building target config $@ succeeded."

## <CONFIG>_rev    : build configured target and add revision to filename
$(addsuffix _rev,$(BASE_CONFIGS)):
	$(V0) $(MAKE) fwo CONFIG=$(subst _rev,,$@) REV=yes
