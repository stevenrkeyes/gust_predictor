Remote Wind Sensor

# Requirements
 - The arm-none-eabi-gcc compiler from xPacks
 - The STM32_Programmer utility from ST Microelectronics

# Build
make clean
# Set your GCC_PATH to wherever you placed the compiler binaries.
# Also, you must specify unique device IDs (0-254) if you want multiple sensors.
make GCC_PATH="~/opt/xPacks/arm-none-eabi-gcc/xpack-arm-none-eabi-gcc-9.3.1-1.1/bin/" DEVICE_ID=0

# Program
# Replace the path with wherever your build folder is
./STM32_Programmer.sh -c port=SWD -w ~/Desktop/code\ for\ wind\ sensor/remote_wind_sensor/build/remote_wind_sensor.hex -hardrst
# If you have multiple sensors, re-build and re-program with each device ID

# Check the debug UART
# Replace the device path with whatever shows up on your machine
od -v --width=1 /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0669FF363336485043073028-if02
