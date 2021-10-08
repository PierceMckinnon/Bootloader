TARGET = bootloader
DEBUG = 1
OPTIMIZE = 0
OPT = -Og
# Build path
BUILD_DIR = ../Bootloader_Build
# C sources
C_SOURCES =  \
./main.c \
./startup.c\
../General_Includes/irq.c \
../Drivers/uart.c \
../Drivers/flash.c \
../Drivers/clk.c \
../Drivers/nvic.c \
../Drivers/gpio.c \
../Drivers/syscfg.c 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-

CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# C includes
C_INCLUDES=\
-I.\
-I../General_Includes/\
-I../Drivers/

CFLAGS = $(MCU) $(C_INCLUDES)  -Wall  -mfloat-abi=hard 

ifeq ($(OPTIMIZE), 1)
CFLAGS+= $(OPT)
endif

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

CFLAGS +=  

# link script
LDSCRIPT = ./stm32f303re.ld

# libraries
LDFLAGS = $(MCU) -nostdlib -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref 

# # default action: build all
# all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
all: $(BUILD_DIR)/$(TARGET).elf


# list of objects
#Add the directory prefix and take away all directory slashes before c sources build/cfile.c
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
#sort directory of the c files build/cfile.c->build/
#tells make when looking for files to look for any files in this directory if not found
vpath %.c $(sort $(dir $(C_SOURCES)))


#$< name of first prereq
#$@ name of current target
#-c compile and do not link, place output in target -o $@
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) $< -o $@ 

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
	#######################################

flash: all
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f3x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
