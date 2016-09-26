#This file is generated by VisualGDB.
#It contains GCC settings automatically derived from the board support package (BSP).
#DO NOT EDIT MANUALLY. THE FILE WILL BE OVERWRITTEN. 
#Use VisualGDB Project Properties dialog or modify Makefile or per-configuration .mak files instead.

#VisualGDB provides BSP_ROOT and TOOLCHAIN_ROOT via environment when running Make. The line below will only be active if GNU Make is started manually.
BSP_ROOT ?= $(LOCALAPPDATA)/VisualGDB/EmbeddedBSPs/arm-eabi/com.sysprogs.arm.mbed
EFP_BASE ?= $(LOCALAPPDATA)/VisualGDB/EmbeddedEFPs
TOOLCHAIN_ROOT ?= C:/SysGCC/arm-eabi

#Embedded toolchain
CC := $(TOOLCHAIN_ROOT)/bin/arm-eabi-gcc.exe
CXX := $(TOOLCHAIN_ROOT)/bin/arm-eabi-g++.exe
LD := $(CXX)
AR := $(TOOLCHAIN_ROOT)/bin/arm-eabi-ar.exe
OBJCOPY := $(TOOLCHAIN_ROOT)/bin/arm-eabi-objcopy.exe

#Additional flags
PREPROCESSOR_MACROS += TOOLCHAIN_GCC TOOLCHAIN_GCC_ARM __MBED__=1 TARGET_RTOS_M4_M7 ARM_MATH_CM4 TARGET_FF_MORPHO MBED_BUILD_TIMESTAMP=1453433829.93 TARGET_NUCLEO_F411RE TARGET_STM TARGET_STM32F411RE __CORTEX_M4 __FPU_PRESENT=1 TARGET_STM32F4 TARGET_CORTEX_M TARGET_M4 TARGET_FF_ARDUINO
INCLUDE_DIRS += $(BSP_ROOT)/libraries/mbed/targets/cmsis/TOOLCHAIN_GCC $(BSP_ROOT)/libraries/mbed/hal $(BSP_ROOT)/libraries/mbed/targets/cmsis $(BSP_ROOT)/libraries/mbed/targets/hal $(BSP_ROOT)/libraries/mbed/common $(BSP_ROOT)/libraries/mbed/targets $(BSP_ROOT)/libraries/mbed $(BSP_ROOT)/libraries/mbed/api $(BSP_ROOT)/libraries/mbed/targets/cmsis/TARGET_STM/TARGET_STM32F4 $(BSP_ROOT)/libraries/mbed/targets/cmsis/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F411RE $(BSP_ROOT)/libraries/mbed/targets/cmsis/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F411RE/TOOLCHAIN_GCC_ARM $(BSP_ROOT)/libraries/mbed/targets/hal/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F411RE $(BSP_ROOT)/libraries/mbed/targets/cmsis/TARGET_STM $(BSP_ROOT)/libraries/mbed/targets/hal/TARGET_STM/TARGET_STM32F4 $(BSP_ROOT)/libraries/mbed/targets/hal/TARGET_STM
LIBRARY_DIRS += 
LIBRARY_NAMES += compactcpp
ADDITIONAL_LINKER_INPUTS += 
MACOS_FRAMEWORKS += 
LINUX_PACKAGES += 

CFLAGS += 
CXXFLAGS += 
ASFLAGS += 
LDFLAGS +=  
COMMONFLAGS += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LINKER_SCRIPT := $(BSP_ROOT)/libraries/mbed/targets/cmsis/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F411RE/TOOLCHAIN_GCC_ARM/STM32F411XE.ld

