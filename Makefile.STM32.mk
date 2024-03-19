SERIAL          := 0673FF333535554157151415
MOUNT           := /mnt/st1
TAG             := s1
BOARD           := NUCLEO_F446RE
EXE_BIN         := stm32-main/BUILD/$(BOARD)/GCC_ARM/stm32-main.bin
LOG_TAG         := "SerialProxy (UART: 1)"
SKIP_COMPILE    ?= 0


define STM32_DefineRules
pTag          := $(1)
pSerial       := $(2)
pMount        := $(3)
pProject      := $(4)
pSkipCompile  := $(5)
pBoard        := $(6)

lProjectName  := $(shell basename $(4))
lExeBin       := $$(pProject)/BUILD/$$(pBoard)/GCC_ARM/$$(lProjectName).bin

$$(info $$(lExeBin))

_MCU   := $$(shell grep -l $$(pSerial) /sys/bus/usb/devices/*/serial)
MCU    := $$(_MCU:/sys/bus/usb/devices/%/serial=%)

_DEV   := $$(shell echo /sys/bus/usb/drivers/usb-storage/$${MCU}*/host*/target*/*:*/block/*)
DEV    := /dev/$$(shell basename $${_DEV})

_ACM   := $$(shell ls /sys/bus/usb/drivers/cdc_acm/$${MCU}*/tty)
ACM    := /dev/$$(shell basename $${_ACM})

$$(pMount):
	sudo mkdir $$(pMount)

$$(pMount)/MBED.HTM: $$(pMount)
	sudo mount $$(DEV) $$(pMount) -o uid=1000,gid=1000


.PHONY: $$(lExeBin)
$$(lExeBin):
	if [ "$$(pSkipCompile)" -eq "1" ]; then \
		echo "Skipped Compile"; \
	else \
		cd /workspaces/korobo2023/stm32-main && \
		mbed compile -m $$(pBoard); \
	fi

$$(pTag)c: $$(lExeBin)

$$(pTag)f: $$(pMount)/MBED.HTM $$(lExeBin)
	cd /workspaces/korobo2023/stm32-main && \
		sudo cp BUILD/$$(pBoard)/GCC_ARM/stm32-main.bin $$(pMount)/binary.bin;
	sudo sync $$(pMount)/binary.bin

$$(pTag)fw: $$(lExeBin)
	./upload_flash.sh $(ESP32_IP) $$(lExeBin)

$$(pTag)rw:
	curl -X POST $(ESP32_IP)/api/stm32/reset

$$(pTag)m:
	cd /workspaces/korobo2023/stm32-main && \
		mbed sterm --port $$(ACM)

$$(pTag)mw:
	ESP32_IP=$(ESP32_IP) python3 /workspaces/korobo2023/ws-relay/log-reader.py $$(LOG_TAG)


endef


$(info $(call STM32_DefineRules,s1,0668FF485151717867193716,/mnt/st1,/workspaces/korobo2023/stm32-main,0,NUCLEO_F446RE))
$(eval $(call STM32_DefineRules,s1,0668FF485151717867193716,/mnt/st1,/workspaces/korobo2023/stm32-main,0,NUCLEO_F446RE))
# $(info $(call STM32_DefineRules,066AFF495057717867162927,s2,/mnt/st2,$(DEV2),stm32-enc/BUILD/$(BOARD)/GCC_ARM/stm32-enc.bin,0,$(BOARD)))

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



rs1w:
	curl -X POST $(ESP32_IP)/api/stm32/reset

ms1:
	cd /workspaces/korobo2023/stm32-main && \
		mbed sterm --port $(ACM1)

ms1w:
	ESP32_IP=$(ESP32_IP) python3 /workspaces/korobo2023/ws-relay/log-reader.py $(LOG_TAG)


ts1: fs1 ms1
a2r1:
	addr2line -e /workspaces/korobo2023/stm32-main/BUILD/NUCLEO_F446RE/GCC_ARM/stm32-main.elf