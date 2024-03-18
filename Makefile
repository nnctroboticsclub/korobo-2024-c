# task runner

# SER1: STM32@Main MCU device serial number
# SER2: STM32@Encoder MCU device serial number

SER1    = 0673FF333535554157151415
# SER1    = 0670FF3932504E3043102150 # F747
SER2    = 0668FF383333554157243840


_MCU1   = $(shell grep -l ${SER1} /sys/bus/usb/devices/*/serial)
MCU1    = $(_MCU1:/sys/bus/usb/devices/%/serial=%)

_MCU2   = $(shell grep -l ${SER2} /sys/bus/usb/devices/*/serial)
MCU2    = $(_MCU2:/sys/bus/usb/devices/%/serial=%)


_DEV1   = $(shell echo /sys/bus/usb/drivers/usb-storage/${MCU1}*/host*/target*/*:*/block/*)
DEV1    = /dev/$(shell basename ${_DEV1})

_DEV2   = $(shell echo /sys/bus/usb/drivers/usb-storage/${MCU2}*/host*/target*/*:*/block/*)
DEV2    = /dev/$(shell basename ${_DEV2})

_ACM1   = $(shell ls /sys/bus/usb/drivers/cdc_acm/${MCU1}*/tty)
ACM1    = /dev/$(shell basename ${_ACM1})

_ACM2   = $(shell ls /sys/bus/usb/drivers/cdc_acm/${MCU2}*/tty)
ACM2    = /dev/$(shell basename ${_ACM2})

MNT1    = /mnt/st1
MNT2    = /mnt/st2

# The ip points localhost. its usually forwarded from VPN.
ESP32_IP ?= localhost:8011

# ESP32 ip address accessed by VPN
REMOTE_ESP32_IP ?= 192.168.0.6

# VPN SSH Address
VPN_SSH := syoch@100.69.175.93

# VPN Commands

# [VPN] Deploy Ws-relay
vdw:
	scp ws-relay/main.py \
			ws-relay/log-reader.py \
			ws-relay/tag-lister.py \
			ws-relay/requirements.txt \
			$(VPN_SSH):/home/syoch/ws-relay/

# [VPN] ForWarD
vfwd:
	ssh \
		-gL 8011:$(REMOTE_ESP32_IP):80 \
		-gL 8012:localhost:8000 \
		$(VPN_SSH)

# [VPN] Reverse ForWarD
vrfwd:
	ssh \
		-gR 0.0.0.0:8050:localhost:8080 \
		$(VPN_SSH)

# [VPN] Start SSH Connection
vpn:
	ssh $(VPN_SSH)


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

$(MNT1):
	sudo mkdir $(MNT1)

$(MNT1)/MBED.HTM: $(MNT1)
	sudo mount $(DEV1) $(MNT1) -o uid=1000,gid=1000

S1BOARD := NUCLEO_F446RE
S1_BIN := stm32-main/BUILD/$(S1BOARD)/GCC_ARM/stm32-main.bin
S1_TAG := "SerialProxy (UART: 1)"
S1_SKIP_COMPILE ?= 0

.PHONY: $(S1_BIN)
ifeq ($(S1_SKIP_COMPILE), 1)
$(S1_BIN):
	@echo "Skipped Compile"
else
$(S1_BIN):
	cd /workspaces/korobo2023/stm32-main && \
		mbed compile -m $(S1BOARD)
endif

cs1: $(S1_BIN)

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

fs1: $(MNT1)/MBED.HTM $(S1_BIN)
	cd /workspaces/korobo2023/stm32-main && \
		sudo cp BUILD/$(S1BOARD)/GCC_ARM/stm32-main.bin $(MNT1)/binary.bin && \
		sudo sync $(MNT1)/binary.bin

fs1w: $(S1_BIN)
	./upload_flash.sh $(ESP32_IP) $(S1_BIN)

rs1w:
	curl -X POST $(ESP32_IP)/api/stm32/reset

ms1:
	cd /workspaces/korobo2023/stm32-main && \
		mbed sterm --port $(ACM1)

ms1w:
	ESP32_IP=$(ESP32_IP) python3 /workspaces/korobo2023/ws-relay/log-reader.py $(S1_TAG)


ts1: fs1 ms1
a2r1:
	addr2line -e /workspaces/korobo2023/stm32-main/BUILD/NUCLEO_F446RE/GCC_ARM/stm32-main.elf


# STM32@Encoder MCU

$(MNT2):
	sudo mkdir $(MNT2)

$(MNT2)/MBED.HTM: $(MNT2)
	sudo mount $(DEV2) $(MNT2) -o uid=1000,gid=1000

cs2:
	cd /workspaces/korobo2023/stm32-enc && \
		mbed compile

fs2: $(MNT2)/MBED.HTM
	cd /workspaces/korobo2023/stm32-enc && \
		sudo cp BUILD/*/GCC_ARM/stm32-enc.bin $(MNT2)/binary.bin && \
		sudo sync $(MNT2)/binary.bin

ms2:
	cd /workspaces/korobo2023/stm32-enc && \
		mbed sterm --port $(ACM2)

ts2: cs2 fs2 ms2
us2: cs2 fs2
a2r2:
	addr2line -e /workspaces/korobo2023/stm32-enc/BUILD/NUCLEO_F446RE/GCC_ARM/stm32-enc.elf

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

tms:
	tmux start

ct: tms
	tmux set-option -g default-terminal screen-256color
	tmux set -g terminal-overrides 'xterm:colors=256'

	tmux set-option -g status-position top
	tmux set-option -g status-left-length 90
	tmux set-option -g status-right-length 90

	tmux set-option -g status-left "#S/#I:#W.#P"
	tmux set-option -g status-right ''
	tmux set-option -g status-justify centre
	tmux set-option -g status-bg "colour238"
	tmux set-option -g status-fg "colour255"

	tmux bind '|' split-window -h
	tmux bind '-' split-window -v

	tmux set-option -g mouse on
	tmux bind -n WheelUpPane if-shell -F -t = "#{mouse_any_flag}" "send-keys -M" "if -Ft= '#{pane_in_mode}' 'send-keys -M' 'copy-mode -e'"

it:
	tmux new-session -d -s korobo2023 -n work

	tmux new-window -t korobo2023:1 -n log
	tmux split-window -t korobo2023:log.0 -v
	tmux split-window -t korobo2023:log.0 -h
	tmux split-window -t korobo2023:log.2 -h
	tmux split-window -t korobo2023:log.3 -h
	tmux resize-pane -t korobo2023:log.1 -x 20

	tmux select-window -t korobo2023:work
	tmux select-window -t korobo2023:log
	tmux send-keys -t korobo2023:log.0 'make me' C-m
	tmux send-keys -t korobo2023:log.1 'make ws' C-m
	tmux send-keys -t korobo2023:log.2 'make ms1' C-m
	tmux send-keys -t korobo2023:log.3 'make ms2' C-m
	tmux send-keys -t korobo2023:log.4 'cd ws-relay; python3 main.py' C-m

at:
	tmux attach -t korobo2023

stm: ct it at
stw:
	tmux new-session -d -s korobo2023-work -n work
	tmux attach -t korobo2023-work
