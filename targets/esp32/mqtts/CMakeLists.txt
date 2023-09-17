
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mqtts)

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "src/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "src/client.key" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "src/mainflux.crt" TEXT)