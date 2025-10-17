
ifeq ($(subst \",,$(CONFIG_HISENSE_PRODUCT_NAME)),)
$(error "Please set CONFIG_HISENSE_PRODUCT_NAME value.")
endif

dtb-y += $(subst $\",,$(CONFIG_HISENSE_PRODUCT_NAME)).dtb

ifeq ($(CONFIG_ARM64),y)
always          := $(dtb-y)
subdir-y        := $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../../, $(dtb-y))

$(obj)/../../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../../,$(dtb-y))
endif

clean-files := *.dtb

