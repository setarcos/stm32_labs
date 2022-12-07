TARGET = hostmsc

CORTEX = cortex-m4
FAMILY = STM32F4
CHIP = STM32F469xx
#CHIP = STM32F469NIHx
BOARD = STM32F469I_DISCO

# optimization
OPT = -Og

#######################################
# paths
#######################################

FIRM_PATH = ../Firmware
HAL_PATH = $(FIRM_PATH)/Drivers/STM32F4xx_HAL_Driver
CMSIS_PATH = $(FIRM_PATH)/Drivers/CMSIS
USBD_PATH = $(FIRM_PATH)/Middlewares/ST/STM32_USB_Host_Library/Core
MSC_PATH = $(FIRM_PATH)/Middlewares/ST/STM32_USB_Host_Library/Class/MSC
FF_PATH = $(FIRM_PATH)/Middlewares/Third_Party/FatFs/src
#######################################
# source files
#######################################
SRC = $(wildcard Src/*.c)
SRC += $(MSC_PATH)/Src/usbh_msc.c
SRC += $(MSC_PATH)/Src/usbh_msc_bot.c
SRC += $(MSC_PATH)/Src/usbh_msc_scsi.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_cortex.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_rcc.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_rcc_ex.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_pwr.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_pwr_ex.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_gpio.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_tim.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_hcd.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_dma.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_adc.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_exti.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_flash.c
SRC += $(HAL_PATH)/Src/stm32f4xx_hal_uart.c
SRC += $(HAL_PATH)/Src/stm32f4xx_ll_usb.c
SRC += $(USBD_PATH)/Src/usbh_core.c
SRC += $(USBD_PATH)/Src/usbh_ctlreq.c
SRC += $(USBD_PATH)/Src/usbh_ioreq.c
SRC += $(USBD_PATH)/Src/usbh_pipes.c
SRC += $(FF_PATH)/ff.c
SRC += $(FF_PATH)/ff_gen_drv.c
SRC += $(FF_PATH)/diskio.c
SRC += $(FF_PATH)/option/unicode.c

# Build path
BUILD_DIR = build

# ASM sources
ASM = ../common/startup_stm32f469xx.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=$(CORTEX)

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS = -DUSE_HAL_DRIVER -D$(CHIP) -D$(FAMILY) -D$(BOARD)

# AS includes
AS_INCLUDES = 

# C includes
INCS = Inc
INCS += $(CMSIS_PATH)/Device/ST/STM32F4xx/Include
INCS += $(CMSIS_PATH)/Include
INCS += $(FIRM_PATH)/Drivers/STM32F4xx_HAL_Driver/Inc
INCS += $(USBD_PATH)/Inc
INCS += $(MSC_PATH)/Inc
INCS += $(FF_PATH)
INCS += USB_DEVICE/Target
INCS += USB_DEVICE/App
INCS += HID/Inc

C_INCLUDES = $(patsubst %,-I%,$(INCS))

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F469NIHx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(SRC:.c=.o)))
vpath %.c $(sort $(dir $(SRC)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM:.s=.o)))
vpath %.s $(sort $(dir $(ASM)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# program
#######################################
prog:
	openocd -d0 -f ../common/openocd.cfg -c init -c "reset halt" -c targets -c "poll off" -c "flash write_image erase build/$(TARGET).elf" -c "verify_image build/$(TARGET).elf" -c "reset run" -c targets -c shutdown

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
