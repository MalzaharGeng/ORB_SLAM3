#!/usr/bin/env bash
# ORB-SLAM3 地图管理：查看 / 切换 / 新建 / 备份 / 关闭复用
# 用法：maps.sh [list|status|use 名字|new 名字|backup|off|rm 名字]
set -u

MAP_ROOT="${MAP_ROOT:-$HOME/maps}"
mkdir -p -- "$MAP_ROOT"

active=""
if [[ -s "$MAP_ROOT/active" ]]; then
    active=$(tr -d '[:space:]' < "$MAP_ROOT/active" 2>/dev/null || true)
fi

size_of() { du -h -- "$1" 2>/dev/null | cut -f1; }

usage() {
    cat <<'EOF'
用法：maps.sh <命令> [参数]

  list              列出所有地图，* 为下次启动会加载的那张
  status            当前使用的地图及其详情
  use <名字>        切换到已有地图（下次启动生效）
  new <名字>        新建地图名，下次启动从零建图并保存到它
  backup            给当前地图打一个带时间戳的快照
  off               关闭复用，下次启动从零建图（不删除任何文件）
  rm <名字>         删除指定地图（会先备份到回收目录）

地图存放：~/maps/<名字>.osa
EOF
}

case "${1:-list}" in
list | l)
    echo "地图目录：$MAP_ROOT"
    if [[ -z "$active" ]]; then
        echo "当前状态：未指定（复用已关闭）"
    else
        echo "当前使用：$active"
    fi
    echo
    printf '%-22s %8s  %-12s %s\n' 名称 大小 修改时间 说明
    shopt -s nullglob
    found=0
    for f in "$MAP_ROOT"/*.osa; do
        found=1
        n=$(basename -- "$f" .osa)
        note=""
        [[ "$n" == "$active" ]] && note="* 下次启动加载"
        printf '%-22s %8s  %-12s %s\n' "$n" "$(size_of "$f")" "$(date -r "$f" '+%m-%d %H:%M')" "$note"
    done
    ((found)) || echo "  （还没有任何地图，下次启动会自动建 default）"
    ;;
status | s)
    if [[ -z "$active" ]]; then
        echo "地图复用：已关闭（下次启动从零建图）"
        exit 0
    fi
    f="$MAP_ROOT/$active.osa"
    echo "当前地图：$active"
    if [[ -s "$f" ]]; then
        printf '  文件：%s\n  大小：%s\n  修改：%s\n' "$f" "$(size_of "$f")" "$(date -r "$f" '+%Y-%m-%d %H:%M:%S')"
        echo "  下次启动：加载这张地图"
    else
        echo "  文件：尚不存在（下次启动从零建图，正常退出后生成）"
    fi
    ;;
use | u)
    name="${2:-}"
    [[ -n "$name" ]] || { echo "用法：maps.sh use <名字>"; exit 1; }
    if [[ ! -s "$MAP_ROOT/$name.osa" ]]; then
        echo "没有这张地图：$MAP_ROOT/$name.osa"
        echo "现有地图："
        shopt -s nullglob
        for f in "$MAP_ROOT"/*.osa; do echo "  $(basename -- "$f" .osa)"; done
        exit 1
    fi
    printf '%s\n' "$name" > "$MAP_ROOT/active"
    echo "已切换：下次启动加载 $name（$(size_of "$MAP_ROOT/$name.osa")）"
    ;;
new | n)
    name="${2:-}"
    [[ -n "$name" ]] || { echo "用法：maps.sh new <名字>"; exit 1; }
    if [[ -s "$MAP_ROOT/$name.osa" ]]; then
        echo "已存在同名地图：$name（不会覆盖，先 use 或换个名字）"
        exit 1
    fi
    printf '%s\n' "$name" > "$MAP_ROOT/active"
    echo "已新建：$name。下次启动从零建图，正常退出后写入 $MAP_ROOT/$name.osa"
    ;;
backup | b)
    [[ -n "$active" && -s "$MAP_ROOT/$active.osa" ]] || { echo "当前没有可备份的地图"; exit 1; }
    dest="$MAP_ROOT/$active.osa.$(date +%Y%m%d_%H%M%S)"
    cp -f -- "$MAP_ROOT/$active.osa" "$dest" && echo "已备份：$dest（$(size_of "$dest")）"
    ;;
off)
    rm -f -- "$MAP_ROOT/active"
    echo "已关闭地图复用。下次启动从零建图，已有的 .osa 文件不会被删除。"
    ;;
rm)
    name="${2:-}"
    [[ -n "$name" ]] || { echo "用法：maps.sh rm <名字>"; exit 1; }
    f="$MAP_ROOT/$name.osa"
    [[ -s "$f" ]] || { echo "没有这张地图：$f"; exit 1; }
    mkdir -p -- "$MAP_ROOT/trash"
    mv -f -- "$f" "$MAP_ROOT/trash/$name.osa.$(date +%Y%m%d_%H%M%S)"
    if [[ "$name" == "$active" ]]; then
        rm -f -- "$MAP_ROOT/active"
        echo "同时关闭了复用（因为删的正是当前地图）"
    fi
    echo "已移到回收目录：$MAP_ROOT/trash/"
    ;;
help | -h | --help)
    usage
    ;;
*)
    usage
    exit 1
    ;;
esac
