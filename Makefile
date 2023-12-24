# task runner

init:
	sudo mount /dev/sdd /mnt -o uid=1000,gid=1000
	exec bash -c ". /opt/esp-idf/export.sh; exec bash"

mon-esp:
	cd /workspaces/korobo2023/esp32 && idf.py monitor

mon-stm-main:
	cd /workspaces/korobo2023/stm32-main && mbed compile -f --sterm