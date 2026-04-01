#!/bin/bash

# CubeMXで生成されたファイルをPlatformIOに対応したディレクトリ構成に変換するスクリプト

# Core/Inc -> include
# Core/Src -> src
# Drivers -> lib/Drivers

set -e  # エラーが発生したら即座に終了

# 色付き出力用
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# ログ関数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ディレクトリ移動関数
# 引数: $1=移動元, $2=移動先
move_directory() {
    local src="$1"
    local dest="$2"

    # 元のディレクトリが存在しない場合はスキップ
    if [ ! -d "$src" ]; then
        log_warn "元のディレクトリが存在しません: $src - スキップします"
        return 0
    fi

    # 元のディレクトリが空の場合はスキップ
    if [ -z "$(ls -A "$src")" ]; then
        log_warn "元のディレクトリが空です: $src - スキップします"
        return 0
    fi

    log_info "処理開始: $src -> $dest"

    # 移動先の親ディレクトリを作成
    local dest_parent=$(dirname "$dest")
    if [ ! -d "$dest_parent" ]; then
        log_info "親ディレクトリを作成: $dest_parent"
        mkdir -p "$dest_parent"
    fi

    # 移動先ディレクトリが既に存在する場合
    if [ -d "$dest" ]; then
        log_info "移動先ディレクトリが既に存在します: $dest"
        log_info "ファイルを個別に移動します..."

        # ファイルを一つずつ移動（上書き確認付き）
        find "$src" -mindepth 1 -maxdepth 1 -print0 | while IFS= read -r -d '' item; do
            local item_name=$(basename "$item")
            if [ -e "$dest/$item_name" ]; then
                log_warn "既に存在します: $dest/$item_name - スキップします"
            else
                log_info "移動: $item_name"
                mv "$item" "$dest/"
            fi
        done
    else
        # 移動先が存在しない場合は、ディレクトリごと移動
        log_info "ディレクトリを移動: $src -> $dest"
        mv "$src" "$dest"
    fi

    # 元のディレクトリが空になったか確認
    if [ -d "$src" ] && [ -z "$(ls -A "$src")" ]; then
        log_info "空のディレクトリを削除: $src"
        rmdir "$src"
    elif [ -d "$src" ]; then
        log_warn "元のディレクトリにまだファイルが残っています: $src"
        log_warn "残っているファイル:"
        ls -la "$src"
    else
        log_info "ディレクトリの移動が完了しました"
    fi

    echo ""
}

# 親ディレクトリを削除する関数（空の場合のみ）
cleanup_empty_parent_dirs() {
    local dir="$1"

    while [ -d "$dir" ] && [ -z "$(ls -A "$dir")" ] && [ "$dir" != "." ] && [ "$dir" != "/" ]; do
        log_info "空の親ディレクトリを削除: $dir"
        rmdir "$dir"
        dir=$(dirname "$dir")
    done
}

# メイン処理
main() {
    log_info "============================================"
    log_info "ディレクトリ移動スクリプト開始"
    log_info "============================================"
    echo ""

    # 1. Core/Inc -> include
    move_directory "./Core/Inc" "./include"

    # 2. Core/Src -> src
    move_directory "./Core/Src" "./src"

    # 3. Drivers -> lib/Drivers
    move_directory "./Drivers" "./lib/Drivers"

    # 空になったCoreディレクトリを削除
    cleanup_empty_parent_dirs "./Core"

    log_info "============================================"
    log_info "すべての処理が完了しました"
    log_info "============================================"
}

# スクリプト実行
main

#!/bin/bash

# ディレクトリ移動スクリプト
# Core/Inc -> include
# Core/Src -> src
# Drivers -> lib/Drivers

set -e  # エラーが発生したら即座に終了

# 色付き出力用
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# ログ関数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ディレクトリ移動関数
# 引数: $1=移動元, $2=移動先
move_directory() {
    local src="$1"
    local dest="$2"

    # 元のディレクトリが存在しない場合はスキップ
    if [ ! -d "$src" ]; then
        log_warn "元のディレクトリが存在しません: $src - スキップします"
        return 0
    fi

    # 元のディレクトリが空の場合はスキップ
    if [ -z "$(ls -A "$src")" ]; then
        log_warn "元のディレクトリが空です: $src - スキップします"
        return 0
    fi

    log_info "処理開始: $src -> $dest"

    # 移動先の親ディレクトリを作成
    local dest_parent=$(dirname "$dest")
    if [ ! -d "$dest_parent" ]; then
        log_info "親ディレクトリを作成: $dest_parent"
        mkdir -p "$dest_parent"
    fi

    # 移動先ディレクトリが既に存在する場合
    if [ -d "$dest" ]; then
        log_info "移動先ディレクトリが既に存在します: $dest"
        log_info "ファイルを個別に移動します..."

        # ファイルを一つずつ移動（上書き確認付き）
        find "$src" -mindepth 1 -maxdepth 1 -print0 | while IFS= read -r -d '' item; do
            local item_name=$(basename "$item")
            if [ -e "$dest/$item_name" ]; then
                log_warn "既に存在します: $dest/$item_name - スキップします"
            else
                log_info "移動: $item_name"
                mv "$item" "$dest/"
            fi
        done
    else
        # 移動先が存在しない場合は、ディレクトリごと移動
        log_info "ディレクトリを移動: $src -> $dest"
        mv "$src" "$dest"
    fi

    # 元のディレクトリが空になったか確認
    if [ -d "$src" ] && [ -z "$(ls -A "$src")" ]; then
        log_info "空のディレクトリを削除: $src"
        rmdir "$src"
    elif [ -d "$src" ]; then
        log_warn "元のディレクトリにまだファイルが残っています: $src"
        log_warn "残っているファイル:"
        ls -la "$src"
    else
        log_info "ディレクトリの移動が完了しました"
    fi

    echo ""
}

# 親ディレクトリを削除する関数（空の場合のみ）
cleanup_empty_parent_dirs() {
    local dir="$1"

    while [ -d "$dir" ] && [ -z "$(ls -A "$dir")" ] && [ "$dir" != "." ] && [ "$dir" != "/" ]; do
        log_info "空の親ディレクトリを削除: $dir"
        rmdir "$dir"
        dir=$(dirname "$dir")
    done
}

# メイン処理
main() {
    log_info "============================================"
    log_info "ディレクトリ移動スクリプト開始"
    log_info "============================================"
    echo ""

    # 1. Core/Inc -> include
    move_directory "./Core/Inc" "./include"

    # 2. Core/Src -> src
    move_directory "./Core/Src" "./src"

    # 3. Drivers -> lib/Drivers
    move_directory "./Drivers" "./lib/Drivers"

    # 空になったCoreディレクトリを削除
    cleanup_empty_parent_dirs "./Core"

    log_info "============================================"
    log_info "すべての処理が完了しました"
    log_info "============================================"
}

# スクリプト実行
main
