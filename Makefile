all:

-include env.home.mk
-include syoch-robotics/makefile.d/all.mk

S1_LOG_TAG      := "SerialProxy (UART: 1)"
S1_SKIP_COMPILE ?= 0

S1_SERIAL       ?= 066AFF495057717867162927

S2_LOG_TAG      := "SerialProxy (UART: 2)"
S2_SKIP_COMPILE ?= 0

ESP_SKIP_COMPILE ?= 0

$(eval $(call STM32_DefineRules,s1,$(ESP32_IP),$(S1_LOG_TAG),/workspaces/korobo2023/stm32-main,$(S1_SKIP_COMPILE),NUCLEO_F446RE,/mnt/st1,$(S1_SERIAL)))
$(eval $(call STM32_DefineRules,s2,$(ESP32_IP),$(S2_LOG_TAG),/workspaces/korobo2023/stm32-enc,$(S2_SKIP_COMPILE),NUCLEO_F446RE,/mnt/st2,066AFF495057717867162927))
$(eval $(call ESP32_DefineRules,e,/workspaces/korobo2023/esp32,$(ESP_SKIP_COMPILE)))

lc:
	wc -ml \
		$$(find ./stm32-main/src -type f) \
		$$(find ./stm32-enc/src -type f) \
		$$(find ./esp32/main -type f) \
		$$(find ./syoch-robotics -type f)