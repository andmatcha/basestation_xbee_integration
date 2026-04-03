PIO ?= pio

UPLINK_DIR := uplink
DOWNLINK_DIR := downlink
UPLINK_ENV := genericSTM32F446RE
DOWNLINK_ENV := genericSTM32F446RE
UPLINK_DEBUG_ENV := genericSTM32F446RE_debug
DOWNLINK_DEBUG_ENV := genericSTM32F446RE_debug

.PHONY: help uplink downlink uplink-debug downlink-debug upload-uplink upload-downlink upload-uplink-debug upload-downlink-debug list info monitor restructure-uplink restructure-downlink

help:
	@printf '%s\n' \
		'make uplink            Upload uplink release build' \
		'make downlink          Upload downlink release build' \
		'make uplink-debug      Upload uplink debug build with printf logs' \
		'make downlink-debug    Upload downlink debug build with printf logs'


# ---------------- 書き込み ----------------
uplink: upload-uplink

downlink: upload-downlink

uplink-debug: upload-uplink-debug

downlink-debug: upload-downlink-debug

upload-uplink:
	$(PIO) run --project-dir $(UPLINK_DIR) -e $(UPLINK_ENV) -t upload

upload-downlink:
	$(PIO) run --project-dir $(DOWNLINK_DIR) -e $(DOWNLINK_ENV) -t upload

upload-uplink-debug:
	$(PIO) run --project-dir $(UPLINK_DIR) -e $(UPLINK_DEBUG_ENV) -t upload

upload-downlink-debug:
	$(PIO) run --project-dir $(DOWNLINK_DIR) -e $(DOWNLINK_DEBUG_ENV) -t upload

# ---------------- デバイスリスト ----------------
list:
	$(PIO) device list

# ---------------- デバイス情報 ----------------
info:
	st-info --probe

# ---------------- モニター ----------------
monitor:
	./f4_swd_monitor.zsh

# ---------------- CubeMX生成コード->PlatformIO ----------------
restructure-uplink:
	cd ./uplink && ./restructure.sh

restructure-downlink:
	cd ./downlink && ./restructure.sh
