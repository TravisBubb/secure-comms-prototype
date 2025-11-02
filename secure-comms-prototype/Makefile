BOARD ?= heltec_wifi_lora_32_V3
PORT ?= /dev/ttyUSB0

PIO = pio
CLANG_FORMAT = clang-format

build:
	@${PIO} run

upload:
	@echo "Uploading to BOARD=${BOARD} on PORT=${PORT}..."
	@${PIO} run --target upload -e ${BOARD} --upload-port ${PORT}
	@echo "Upload complete."

monitor:
	@echo "Starting serial monitor for BOARD=${BOARD} on PORT=${PORT}..."
	@${PIO} device monitor --port ${PORT}

upload-and-monitor: upload monitor

format:
	@echo "Formatting source code..."
	@${CLANG_FORMAT} -i **/*.cpp **/*.h
	@echo "Formatting complete."

.PHONY: build upload monitor upload-and-monitor format