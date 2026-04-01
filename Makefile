PIO ?= pio

UPLINK_DIR := uplink
DOWNLINK_DIR := downlink

.PHONY: help uplink downlink upload-uplink upload-downlink

help:
	@printf '%s\n' \
		'make uplink      Run `pio run -t upload` for uplink' \
		'make downlink    Run `pio run -t upload` for downlink'


# ---------------- 書き込み ----------------
uplink: upload-uplink

downlink: upload-downlink

upload-uplink:
	$(PIO) run --project-dir $(UPLINK_DIR) -t upload

upload-downlink:
	$(PIO) run --project-dir $(DOWNLINK_DIR) -t upload

# ---------------- デバイスリスト ----------------
list:
	$(PIO) device list

# ---------------- モニター ----------------
monitor:
	./f4_swd_monitor.zsh
