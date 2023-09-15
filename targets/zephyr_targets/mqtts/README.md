MQTTS- stm32f4 target
## Requirements
1. Mainflux broker details including: hostname, ThingID, Thing Credentials and Channel ID
2. [Zephyr](https://www.zephyrproject.org/)


## Configure
1. Edit the [config file](include/config.h) with your broker details.

## Build
The project can be built by utilising the make file within the target directory

```bash
west build -p always -b <your-board-name> mqtt
```
## Flash
Platform io generate a build directory with the fimware.bin within it. Use the make command to flash to board
```bash
west flash
```