SKETCH_DIR := main
BUILD_DIR := build
ARDUINO_CLI := arduino-cli
ESPTOOL := /usr/bin/esptool

FQBN := esp32:esp32:heltec_wifi_lora_32_V3
DEVICE_PORT ?= /dev/ttyUSB0
CHIP := esp32s3
BAUD := 921600

ARDUINO_FLAGS := --fqbn $(FQBN) $(SKETCH_DIR)

BOOT_APP0 := $(shell find ~/.arduino15/packages/esp32/hardware/esp32/ -name boot_app0.bin | sort | tail -n1)

all: build

build: clean
	@echo "Building sketch in $(BUILD_DIR)..."
	$(ARDUINO_CLI) compile --clean $(ARDUINO_FLAGS) --build-path $(BUILD_DIR)

# Fast sketch-only upload (use this most of the time)
flash-app: build
	@echo "Flashing application to $(DEVICE_PORT)..."
	$(ESPTOOL) --chip $(CHIP) --port $(DEVICE_PORT) --baud $(BAUD) \
		--before default-reset \
		--after hard-reset write-flash -z \
		0x10000 $(BUILD_DIR)/main.ino.bin

# Full clean build + flash everything (rarely needed)
flash-full: build
	@echo "Flashing bootloader, partitions, boot_app0, and application to $(DEVICE_PORT)..."
	$(ESPTOOL) --chip $(CHIP) --port $(DEVICE_PORT) --baud $(BAUD) \
		--before default-reset \
		--after hard-reset write-flash -z \
		--flash-mode keep --flash-freq keep --flash-size keep \
		0x0 $(BUILD_DIR)/main.ino.bootloader.bin \
		0x8000 $(BUILD_DIR)/main.ino.partitions.bin \
		0xe000 $(BOOT_APP0) \
		0x10000 $(BUILD_DIR)/main.ino.bin

clean:
	@echo "Cleaning build directory..."
	@rm -rf $(BUILD_DIR) ~/.arduino15/cache ~/.arduino15/staging

.PHONY: all clean flash-app flash-full
