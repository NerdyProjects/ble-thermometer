BIN = bodythermometer
NRG_LIB_DIR = ../Library/BlueNRG-3.1.0/Library
DEF := -DSENSOR_EMULATION -DST_OTA_HIGHER_APPLICATION -DLS_SOURCE=LS_SOURCE_EXTERNAL_32KHZ -DSMPS_INDUCTOR=SMPS_INDUCTOR_10uH -DBLUENRG1_DEVICE -DHS_SPEED_XTAL=HS_SPEED_XTAL_32MHZ

# Library throws a lot of warnings - silence them a bit
WARNFLAGS = -Wall -Wextra -Wimplicit-function-declaration

INC= -I$(NRG_LIB_DIR)/hal/inc \
-I$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/inc \
-I$(NRG_LIB_DIR)/CMSIS/Device/ST/BlueNRG1/Include \
-I$(NRG_LIB_DIR)/CMSIS/Include \
-I$(NRG_LIB_DIR)/Bluetooth_LE/inc \
-I$(NRG_LIB_DIR)/BLE_Application/layers_inc \
-I$(NRG_LIB_DIR)/BLE_Application/Utils/inc \
-I$(NRG_LIB_DIR)/BLE_Application/OTA/inc \

SRC_C = main.c \
BlueNRG1_it.c \
$(NRG_LIB_DIR)/BLE_Application/OTA/src/OTA_btl.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_adc.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_adc.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_sysCtrl.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/misc.c \
$(NRG_LIB_DIR)/hal/src/sleep.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_i2c.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_pka.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_spi.c \
$(NRG_LIB_DIR)/Bluetooth_LE/src/stack_user_cfg.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_mft.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_uart.c \
$(NRG_LIB_DIR)/hal/src/osal.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_gpio.c \
$(NRG_LIB_DIR)/CMSIS/Device/ST/BlueNRG1/Source/system_bluenrg1.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_dma.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_rtc.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_wdg.c \
$(NRG_LIB_DIR)/BlueNRG1_Periph_Driver/src/BlueNRG1_flash.c \
$(NRG_LIB_DIR)/hal/src/clock.c \
$(NRG_LIB_DIR)/hal/src/miscutil.c \

SRC_ASM = context_switch.s

SRC_LD = BlueNRG1.ld

# OpenOCD setup
OPENOCD ?= openocd -f board/st_nucleo_f0.cfg

include ../cm-makefile/config.mk
LDFLAGS = $(ARCHFLAGS) $(OPTFLAGS) $(DBGFLAGS) -lbluenrg1_stack -lcrypto -mthumb -specs=nosys.specs -nostartfiles -nodefaultlibs -static -L$(NRG_LIB_DIR)/Bluetooth_LE/library -L$(NRG_LIB_DIR)/cryptolib -Wl,--cref -Wl,-Map=$(BUILD_DIR)/$(BIN).map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80 -Wl,--start-group -lc -lm -Wl,--end-group -specs=nano.specs
#arm-atollic-eabi-gcc -c -mthumb -g -Wa,--no-warn -x assembler-with-cpp -specs=nano.specs -o HAL/context_switch.o
$(BUILD_DIR)/context_switch.o: $(NRG_LIB_DIR)/hal/src/context_switch.s
	@echo "  AS      $(notdir $@)"
	$(CMD_ECHO) $(CC) -c -mthumb -g -Wa,--no-warn -x assembler-with-cpp -specs=nano.specs -o $@ $<
include ../cm-makefile/openocd.mk
include ../cm-makefile/rules.mk

#-lbluenrg1_stack -lcrypto -mthumb -T../BlueNRG1.ld -specs=nosys.specs -nostartfiles -nodefaultlibs -static -L../../../../../../Library/Bluetooth_LE/library -L../../../../../../Library/cryptolib -Wl,--cref -Wl,-Map=BLE_SensorDemo.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80 -Wl,--start-group -lc -lm -Wl,--end-group -specs=nano.specs 

#arm-atollic-eabi-gcc -o BLE_SensorDemo.elf HAL/clock.o HAL/context_switch.o HAL/miscutil.o HAL/osal.o HAL/sleep.o Library/BlueNRG1_adc.o Library/BlueNRG1_dma.o Library/BlueNRG1_flash.o Library/BlueNRG1_gpio.o Library/BlueNRG1_i2c.o Library/BlueNRG1_mft.o Library/BlueNRG1_pka.o Library/BlueNRG1_rtc.o Library/BlueNRG1_spi.o Library/BlueNRG1_sysCtrl.o Library/BlueNRG1_uart.o Library/BlueNRG1_wdg.o Library/misc.o Platform_Layers/LPS25HB.o Platform_Layers/LSM6DS3.o Platform_Layers/SDK_EVAL_Button.o Platform_Layers/SDK_EVAL_Com.o Platform_Layers/SDK_EVAL_Config.o Platform_Layers/SDK_EVAL_I2C.o Platform_Layers/SDK_EVAL_Led.o Platform_Layers/SDK_EVAL_SPI.o Stack/stack_user_cfg.o Startup/system_bluenrg1.o User/BlueNRG1_it.o User/SensorDemo_main.o User/gatt_db.o User/sensor.o -lbluenrg1_stack -lcrypto -mthumb -T../BlueNRG1.ld -specs=nosys.specs -nostartfiles -nodefaultlibs -static -L../../../../../../Library/Bluetooth_LE/library -L../../../../../../Library/cryptolib -Wl,--cref -Wl,-Map=BLE_SensorDemo.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80 -Wl,--start-group -lc -lm -Wl,--end-group -specs=nano.specs 
