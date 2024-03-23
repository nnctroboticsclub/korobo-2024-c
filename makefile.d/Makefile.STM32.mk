ESP32_IP        ?= 192.168.1.114

STM32_ResolveMountpointFromTag = /mnt/st-$(1)

STM32_ResolvUSBIdFromSerial_ = $(shell grep -l $(1) /sys/bus/usb/devices/*/serial)
STM32_ResolvUSBIdFromSerial = $(STM32_ResolvUSBIdFromSerial_:/sys/bus/usb/devices/%/serial=%)

STM32_ResolvDeviceFromUSBId_ = $(shell echo /sys/bus/usb/drivers/usb-storage/$(1)*/host*/target*/*:*/block/*)
STM32_ResolvDeviceFromUSBId = /dev/$(shell basename $(call STM32_ResolvDeviceFromUSBId_,$(1)))
STM32_ResolvDeviceFromSerial = $(call STM32_ResolvDeviceFromUSBId,$(call STM32_ResolvUSBIdFromSerial,$(1)))

STM32_ResolvACMFromUSBId_ = $(shell ls /sys/bus/usb/drivers/cdc_acm/$(1)*/tty)
STM32_ResolvACMFromUSBId = /dev/$(shell basename $(call STM32_ResolvACMFromUSBId_,$(1)))
STM32_ResolvACMFromSerial = $(call STM32_ResolvACMFromUSBId,$(call STM32_ResolvUSBIdFromSerial,$(1)))

# (project_dir)
STM32_ResolvProjectName = $(shell basename $(1))

# (project, board)
STM32_ResolvExebin = $(1)/BUILD/$(2)/GCC_ARM/$(call STM32_ResolvProjectName,$(1)).bin

# (mount_point)
define STM32_DefineRule_MountPoint
$(1):
	sudo mkdir $(1)

endef

# (mount_point, storage_device)
define STM32_DefineRule_Mount
$(1)/MBED.HTM: $(1)
	sudo mount $(2) $(1) -o uid=1000,gid=1000

endef

# (tag, exe_bin, project_dir, skip_compile, board)
define STM32_DefineRule_Compile
.PHONY: $(2)
$(2):
ifneq ($(4), 0)
		@printf "\e[92mSkipped $(shell basename $(3))'s binary Compile\e[m\n"
else
	cd $(3) && mbed compile -m $(5)
endif

$(1)c: $(2)

endef

# (tag, mount_point, exe_bin)
define STM32_DefineRule_LocalFlash
$(1)f: $(3) $(2)/MBED.HTM
	sudo cp $(3) $(2)/binary.bin
	sudo sync $(2)/binary.bin

endef

# (tag, esp32_ip, exe_bin)
define STM32_DefineRule_RemoteFlash
$(1)fw: $(3)
	./upload_flash.sh $(2) $(3)

endef

# (tag, acm_device)
define STM32_DefineRule_LocalMonitor
$(1)m:
	mbed sterm --port $(2)

endef

# (tag, esp32_ip, log_tag)
define STM32_DefineRule_RemoteMonitor
$(1)mw:
	ESP32_IP=$(2) python3 ws-relay/log-reader.py $(3)

endef

# (tag, esp32_ip)
define STM32_DefineRule_RemoteReset
$(1)rw:
	curl -X POST $(2)/api/stm32/reset

endef

# (tag, esp32_ip, exe_bin, log_tag)
define STM32_DefineRulesRemote
$(call STM32_DefineRule_RemoteFlash,$(1),$(2),$(3))
$(call STM32_DefineRule_RemoteMonitor,$(1),$(2),$(4))
$(call STM32_DefineRule_RemoteReset,$(1),$(2))
endef

# (tag, exe_bin, project_dir, skip_compile, board, mount_point, acm_device)
define STM32_DefineRulesLocal
$(call STM32_DefineRule_Compile,$(1),$(2),$(3),$(4),$(5))
$(call STM32_DefineRule_LocalFlash,$(1),$(6),$(2))
$(call STM32_DefineRule_LocalMonitor,$(1),$(7))
endef

# (mount_point, storage_device)
define STM32_DefineRulesMount
$(call STM32_DefineRule_MountPoint,$(1))
$(call STM32_DefineRule_Mount,$(1),$(2))
endef

# (tag, esp32_ip, exe_bin, log_tag,
#   project_dir, skip_compile, board, mount_point,
#   acm_device, storage_device)
define STM32_DefineRules_
$(call STM32_DefineRulesRemote,$(1),$(2),$(3),$(4))
$(call STM32_DefineRulesLocal,$(1),$(3),$(5),$(6),$(7),$(8),$(9))
$(call STM32_DefineRulesMount,$(8),$(10))
endef

# (tag, esp32_ip, log_tag, project_dir,
#   skip_compile, board, mount_point, usb_serial)
define STM32_DefineRules
$(call STM32_DefineRules_,$(1),$(2),$(call STM32_ResolvExebin,$(4),$(6)),$(3),$(4),$(5),$(6),$(7),$(call STM32_ResolvACMFromSerial,$(8)),$(call STM32_ResolvDeviceFromSerial,$(8)))
endef

cs1t:
	g++ \
	  -I ./emu/stm32 \
		$(addprefix -I, $(shell find /workspaces/korobo2023/stm32-main \
			-not -path *mbed-os* -a \
			-not -path *BUILD* -a \
			-not -path *.git* -a \
			-not -path *.hg* -a \
			-not -path *src* -a \
			-type d \
		)) \
		$(shell find /workspaces/korobo2023/stm32-main/src \
			\( \
				-name *.c -o \
				-name *.cpp \
			\) -a \
			-not -path \*mbed-os\* \
		) \
		$(shell find /workspaces/korobo2023/emu/stm32 \
			-name *.c -o \
			-name *.cpp \
		) \
		-o stm32-main.elf

a2r1:
	addr2line -e /workspaces/korobo2023/stm32-main/BUILD/NUCLEO_F446RE/GCC_ARM/stm32-main.elf