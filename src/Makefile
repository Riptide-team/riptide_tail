ARDUINO_DIR = /opt/arduino-1.8.19
ARDMK_DIR = /opt/arduino-mk

BOARD_TAG = nano
BOARD_SUB = atmega328
MONITOR_PORT  = /dev/ttyUSB0

ARDUINO_SKETCHBOOK = $(realpath $(dir $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))))
USER_LIB_PATH = $(ARDUINO_SKETCHBOOK)/libraries
ARDUINO_LIBS += PPMReader Servo NMEAParser

include ${ARDMK_DIR}/Arduino.mk