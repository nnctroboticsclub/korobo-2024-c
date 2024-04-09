all:

-include env.home.mk
-include syoch-robotics/makefile.d/all.mk

S1_LOG_TAG      := "SerialProxy (UART: 1)"
S1_SKIP_COMPILE ?= 0

S2_LOG_TAG      := "SerialProxy (UART: 2)"
S2_SKIP_COMPILE ?= 0

ESP_SKIP_COMPILE ?= 0

$(eval $(call STM32_DefineRules,s1,$(ESP32_IP),$(S1_LOG_TAG),/workspaces/korobo2023/stm32-main,$(S1_SKIP_COMPILE),NUCLEO_F446RE,/mnt/st1,066EFF303435554157121019))
$(eval $(call STM32_DefineRules,s2,$(ESP32_IP),$(S2_LOG_TAG),/workspaces/korobo2023/stm32-enc,$(S2_SKIP_COMPILE),NUCLEO_F446RE,/mnt/st2,066AFF495057717867162927))
$(eval $(call ESP32_DefineRules,e,/workspaces/korobo2023/esp32,$(ESP_SKIP_COMPILE)))