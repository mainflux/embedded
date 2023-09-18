# MQTTS- stm32 target
## Requirements
1. Mainflux broker details including: hostname, ThingID, Thing Credentials and Channel ID
2. [PlatformIO](https://platformio.org/)
3. [dfu-util](https://dfu-util.sourceforge.net/)
4. [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

## Configure
Resolving nucleo_f429zi dependencies...
1. Use the STM32CUbeIDE to generate the specific files for your target. Please ensure to add the [Lwip](https://git.savannah.nongnu.org/git/lwip.git) and [Paho embedded c](https://github.com/eclipse/paho.mqtt.embedded-c.git) amd [mbedtls](https://github.com/Mbed-TLS/mbedtls) libraries as third party libraries. Then copy the files to this section
Edit the platform.ini file for the specific target.
2. Edit the [config file](include/config.h) with your broker and network details.
3. Generate and add a certificate in the [MQTTINterface source file](src\MQTTInterface.c).

## Build
The project can be built by utilising the make file within the target directory

```bash
make 
```
## Flash
Platform io generate a build directory with the fimware.bin within it. Use the make command to flash to board
```bash
make upload
```

