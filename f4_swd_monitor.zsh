#!/usr/bin/env zsh
set -euo pipefail

# ソースコードに記述したprintfの出力を表示するためのスクリプト

# OpenOCD をバックグラウンドで起動し、nc localhost 3344 を実行する
# 終了時に OpenOCD を確実に停止します

OPENOCD_CMD=(
  openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "init" \
    -c "stm32f4x.tpiu configure -protocol uart -traceclk 168000000 -pin-freq 1000000 -output :3344 -formatter off" \
    -c "itm ports on" \
    -c "stm32f4x.tpiu enable" \
    -c "reset run"
)

LOG_FILE="/tmp/openocd_itm_$(date +%Y%m%d_%H%M%S).log"

echo "[run_itm_nc] OpenOCD をバックグラウンドで起動します... (log: $LOG_FILE)" >&2
"${OPENOCD_CMD[@]}" >"$LOG_FILE" 2>&1 &
OPENOCD_PID=$!

cleanup() {
  if kill -0 "$OPENOCD_PID" 2>/dev/null; then
    echo "[run_itm_nc] OpenOCD (PID: $OPENOCD_PID) を停止します" >&2
    kill "$OPENOCD_PID" 2>/dev/null || true
    wait "$OPENOCD_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# ポート 3344 が開くまで短時間待機
echo "[run_itm_nc] ポート 3344 を待機中..." >&2
for i in {1..100}; do
  if nc -z localhost 3344 2>/dev/null; then
    break
  fi
  sleep 0.1
done

echo "[run_itm_nc] 接続開始: nc localhost 3344" >&2
nc localhost 3344

echo "[run_itm_nc] nc 終了。クリーンアップします。" >&2
