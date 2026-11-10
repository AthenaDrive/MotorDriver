# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# This firmware targets the AthenaDrive v3 ESP32-S3 board. Pin it here so a
# fresh clone builds for the right target without a local sdkconfig.
set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(AthenaDrive)
