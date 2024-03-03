# task runner

# SER1: STM32@Main MCU device serial number
# SER2: STM32@Encoder MCU device serial number

# SER1    = 066BFF303435554157043738
SER1    = 0670FF3932504E3043102150 # F747
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

i:
	[ ! -e /usr/local/bin/websocat ] && sudo cp /workspaces/korobo2023/websocat.x86_64-unknown-linux-musl /usr/local/bin/websocat; true

	mbed config -G GCC_ARM_PATH /opt/gcc-arm-none-eabi-10.3-2021.10/bin
	mbed toolchain -G GCC_ARM

	[ -z $${IDF_PATH+x} ] && exec bash -c ". /opt/esp-idf/export.sh; exec bash"; true

im:
	cd stm32-main; mbed deploy

dns:
	sudo bash -c 'echo nameserver 8.8.8.8 > /etc/resolv.conf'

d:
	sudo umount $(MNT1)
	sudo umount $(MNT2)

	sudo rm -rf $(MNT1)
	sudo rm -rf $(MNT2)

	[ -e /usr/local/bin/websocat ] && sudo rm /usr/local/bin/websocat; true

me:
	cd /workspaces/korobo2023/esp32 && idf.py monitor

# STM32@Main MCU

$(MNT1):
	sudo mkdir $(MNT1)

$(MNT1)/MBED.HTM: $(MNT1)
	sudo mount $(DEV1) $(MNT1) -o uid=1000,gid=1000

cs1:
	cd /workspaces/korobo2023/stm32-main && \
		mbed compile --profile mbed-os/tools/profiles/debug.json -m NUCLEO_F767ZI

fs1: $(MNT1)/MBED.HTM
	cd /workspaces/korobo2023/stm32-main && \
		sudo cp BUILD/*/GCC_ARM-DEBUG/stm32-main.bin $(MNT1)/binary.bin && \
		sudo sync $(MNT1)/binary.bin

ws1:
	./upload_flash.sh \
		localhost:8011 \
		stm32-main/BUILD/NUCLEO_F446RE/GCC_ARM-DEBUG/stm32-main.bin

ms1:
	cd /workspaces/korobo2023/stm32-main && \
		mbed sterm --port $(ACM1)


ts1: cs1 fs1 ms1
us1: cs1 fs1
rs1: cs1 ws1
a2r1:
	addr2line -e /workspaces/korobo2023/stm32-main/BUILD/NUCLEO_F446RE/GCC_ARM-DEBUG/stm32-main.elf


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
	wc -l $$(find . \
		-not \( \
			-path *QEI2_os6* \
			-o -path *ikakoMDC* \
			-o -path ./stm32-main/bno055 \
			-o -path *build* \
			-o -path *ikarashiCAN_mk2* \
			-o -path *BUILD/NUCLEO_F446RE* \
			-o -path *managed_components* \
			-o -path *mbed-os* \
		\) -a \
		\( \
			-name *.hpp \
			-o -name *.cpp \
			-o -name *.c \
			-o -name *.h \
		\))

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