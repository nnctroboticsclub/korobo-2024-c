all:
	echo -

-include Makefile.vpn.mk
-include Makefile.tmux.mk
-include Makefile.STM32.mk

S1_LOG_TAG      := "SerialProxy (UART: 1)"
S1_SKIP_COMPILE ?= 0

S2_LOG_TAG      := "SerialProxy (UART: 2)"
S2_SKIP_COMPILE ?= 0

# The ip points localhost. its usually forwarded from VPN.
ESP32_IP ?= localhost:8011

# ESP32 ip address accessed by VPN
REMOTE_ESP32_IP ?= 192.168.0.6

# VPN SSH Address
VPN_SSH := syoch@100.69.175.93

$(info $(call STM32_DefineRules,s1,$(ESP32_IP),$(S1_LOG_TAG),/workspaces/korobo2023/stm32-main,$(S1_SKIP_COMPILE),NUCLEO_F446RE,/mnt/st1,066EFF303435554157121019))
# $(eval $(call STM32_DefineRules,s1,066EFF303435554157121019,/mnt/st1,/workspaces/korobo2023/stm32-main,$(S1_SKIP_COMPILE),NUCLEO_F446RE,$(S1_LOG_TAG)))

# $(info $(call STM32_DefineRules,s2,066AFF495057717867162927,/mnt/st2,/workspaces/korobo2023/stm32-enc,$(S2_SKIP_COMPILE),NUCLEO_F446RE,$(S2_LOG_TAG)))
# $(eval $(call STM32_DefineRules,s2,066AFF495057717867162927,/mnt/st2,/workspaces/korobo2023/stm32-enc,$(S2_SKIP_COMPILE),NUCLEO_F446RE,$(S2_LOG_TAG)))








# [Init] Container
i_c:
	sudo apt update
	sudo apt install -y tmux

# [Init] General
i:
	[ ! -e /usr/local/bin/websocat ] && sudo cp /workspaces/korobo2023/websocat.x86_64-unknown-linux-musl /usr/local/bin/websocat; true

	mbed config -G GCC_ARM_PATH /opt/gcc-arm-none-eabi-10.3-2021.10/bin
	mbed toolchain -G GCC_ARM

	[ -z $${IDF_PATH+x} ] && exec bash -c ". /opt/esp-idf/export.sh; exec bash"; true


# [Init] Mbed
im:
	cd stm32-main; mbed deploy


# [Init]
id:
	sudo bash -c 'echo nameserver 8.8.8.8 > /etc/resolv.conf'

d:
	sudo umount $(MNT1)
	sudo umount $(MNT2)

	sudo rm -rf $(MNT1)
	sudo rm -rf $(MNT2)

	[ -e /usr/local/bin/websocat ] && sudo rm /usr/local/bin/websocat; true

# ESP32

me:
	cd /workspaces/korobo2023/esp32 && ESPBAUD=960000 idf.py monitor

mew:
	ESP32_IP=$(ESP32_IP) python3 /workspaces/korobo2023/ws-relay/log-reader.py "ESP32"

# STM32@Main MCU




a2r1:
	addr2line -e /workspaces/korobo2023/stm32-main/BUILD/NUCLEO_F446RE/GCC_ARM/stm32-main.elf

# NAS

NAS := /mnt/RoboNAS
NAS_FLASH := $(NAS)/flash
NAS_FLASH_KOROBO2024C := $(NAS_FLASH)/korobo-2024-C
NAS_FLASH_MDC := $(NAS_FLASH)/mdc

$(NAS):
	sudo mkdir $(NAS)

$(NAS)/aquota.group: $(NAS)
	@if mount | grep /mnt/RoboNAS 2>&1 1>/dev/null; then \
		echo "Already Mounted"; \
	else \
		sudo mount //100.69.175.93/Public $(NAS) -t cifs \
			-o username=robo,password=robo,iocharset=utf8,file_mode=0777,dir_mode=077; \
		echo "Mounted"; \
	fi

$(NAS_FLASH): $(NAS)/aquota.group
	[ -d $(NAS_FLASH) ] || sudo mkdir $(NAS_FLASH)

$(NAS_FLASH_KOROBO2024C): $(NAS_FLASH)
	[ -d $(NAS_FLASH_KOROBO2024C) ] || sudo mkdir $(NAS_FLASH_KOROBO2024C)

$(NAS_FLASH_MDC): $(NAS_FLASH)
	[ -d $(NAS_FLASH_MDC) ] || sudo mkdir $(NAS_FLASH_MDC)

ns1: cs1 $(NAS_FLASH_KOROBO2024C)
	cp $(S1_BIN) $(NAS_FLASH_KOROBO2024C)/stm32-main-$(shell date +%Y%m%d-%H%M%S).bin

ns2: cs2 $(NAS_FLASH_KOROBO2024C)
	cp /workspaces/korobo2023/stm32-enc/BUILD/*/GCC_ARM/stm32-enc.bin $(NAS_FLASH_KOROBO2024C)/stm32-enc-$(shell date +%Y%m%d-%H%M%S).bin

nsn: $(NAS_FLASH_MDC)
	cp /workspaces/nishiwakiMDC/BUILD/NUCLEO_F446RE/GCC_ARM/nishiwakiMDC.bin $(NAS_FLASH_MDC)/nishiwakiMDC-$(shell date +%Y%m%d-%H%M%S).bin


# Some useful commands

lu:
	@{ for D in /sys/bus/usb/devices/*; do \
		printf "%s:%s on %s-%s (%32s) :: %s\n" \
			$$(cat $$D/idVendor) $$(cat $$D/idProduct) \
			$$(cat $$D/busnum) $$(cat $$D/devpath) \
			$$(cat $$D/serial) \
			"$$(cat $$D/product)"; \
  done; } 2>/dev/null | grep -v "^: on -"

ws:
	watch -n 1 ls /mnt/st*

lc:
	wc -l \
		$$(find ./stm32-main/src -type f)


# Tmux


