#Generated by VisualGDB (http://visualgdb.com)
#DO NOT EDIT THIS FILE MANUALLY UNLESS YOU ABSOLUTELY NEED TO
#USE VISUALGDB PROJECT PROPERTIES DIALOG INSTEAD

BINARYDIR := Debug

#Additional flags
PREPROCESSOR_MACROS := DEBUG=1 USE_STM32F4XX_NUCLEO STM32F411xE OSXMOTIONFX_STORE_CALIB_FLASH
INCLUDE_DIRS := src src/ble src/imu ../../../../../../../STM32/X-CUBE-BLE1/Drivers/BSP/X-NUCLEO-IDB04A1 ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/LowPowerManager/Inc ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/STM32_BlueNRG/Interface ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/STM32_BlueNRG/STM32F4xx_HAL_BlueNRG_Drivers/inc ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/TimerServer/inc ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/TimerServer/STM32xx_HAL_TimerServer_Drivers/inc ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/includes ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/hci ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/hci/controller ../../../../../../../STM32/X-CUBE-BLE1/Middlewares/ST/STM32_BlueNRG/SimpleBlueNRG_HCI/utils ../../../../../../../STM32/X-CUBE-BLE1/Drivers/BSP/STM32F4xx-Nucleo ../../../../../../../STM32/X-CUBE-MEMS1/Drivers/BSP/Components/Common ../../../../../../../STM32/X-CUBE-MEMS1/Drivers/BSP/Components/hts221 ../../../../../../../STM32/X-CUBE-MEMS1/Drivers/BSP/Components/lis3mdl ../../../../../../../STM32/X-CUBE-MEMS1/Drivers/BSP/Components/lps22hb ../../../../../../../STM32/X-CUBE-MEMS1/Drivers/BSP/Components/lps25hb ../../../../../../../STM32/X-CUBE-MEMS1/Drivers/BSP/Components/lsm6ds0 ../../../../../../../STM32/X-CUBE-MEMS1/Drivers/BSP/Components/lsm6ds3 ../../../../../../../STM32/X-CUBE-MEMS1/Drivers/BSP/X_NUCLEO_IKS01A1 ../../../../../../../STM32/X-CUBE-MEMS1/Middlewares/ST/STM32_OSX_MotionFX_Library ../../../../../../../STM32/X-CUBE-MEMS1/Middlewares/ST/STM32_OSX_MotionFX_Library/Inc ../../../../../../../STM32/X-CUBE-WIFI1/Middlewares/ST/STM32_SPWF01SA/Inc ../../../../../../../STM32/X-CUBE-WIFI1/Middlewares/ST/STM32_SPWF01SA/Utils ../../../../../../../STM32/X-CUBE-WIFI1/Middlewares/ST/STM32_SPWF01SA ../../../../../../../STM32/X-CUBE-WIFI1/Drivers/BSP/X-NUCLEO-IDW01M1 src/motor src/math src/Controller src/PID src/wifi
LIBRARY_DIRS := ../../../../../../../STM32/X-CUBE-MEMS1/Middlewares/ST/STM32_OSX_MotionFX_Library/Lib
LIBRARY_NAMES := osxMotionFX107_CM4_GCC_ot
ADDITIONAL_LINKER_INPUTS := 
MACOS_FRAMEWORKS := 
LINUX_PACKAGES := 

CFLAGS := -ggdb -ffunction-sections -O0
CXXFLAGS := -ggdb -ffunction-sections -fno-exceptions -fno-rtti -O0
ASFLAGS := 
LDFLAGS := -Wl,-gc-sections
COMMONFLAGS := 
LINKER_SCRIPT := 

START_GROUP := 
END_GROUP := 

#Additional options detected from testing the toolchain
USE_DEL_TO_CLEAN := 1
CP_NOT_AVAILABLE := 1

ADDITIONAL_MAKE_FILES := mbed.mak
GENERATE_BIN_FILE := 1
GENERATE_IHEX_FILE := 0
