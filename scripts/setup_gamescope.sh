#!/bin/sh
# Поставить на машину со старым ботом всё, что нужно новым обновлениям:
# gamescope (окно-композит), libei (виртуальные клавиатура и мышь внутри него),
# Python-окружение и доступ к памяти своего процесса CS2.
#
# Запуск от обычного пользователя, не от root:
#   bash scripts/setup_gamescope.sh
#
# Скрипт не запускает игру и не трогает параметры мыши.
# В поле Launch Options у CS2 в Steam ничего писать не нужно:
# бот сам поднимает gamescope снаружи snap.

set -eu

if [ "$(id -u)" -eq 0 ]; then
    echo "Запусти от своего пользователя: bash scripts/setup_gamescope.sh" >&2
    exit 1
fi

ROOT=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
cd "$ROOT"

echo "== репозиторий gamescope =="
sudo apt-get update
sudo apt-get install -y software-properties-common ca-certificates
. /etc/os-release
case "${VERSION_CODENAME:-}" in
    noble) ;;
    *)
        echo "Репозиторий gamescope собран для Ubuntu 24.04 (noble)." >&2
        echo "Сейчас система: ${VERSION_CODENAME:-неизвестно}." >&2
        exit 1
        ;;
esac
sudo add-apt-repository -y ppa:3v1n0/gamescope

echo "== пакеты =="
sudo apt-get update
sudo apt-get install -y \
    gamescope \
    libei1 \
    ffmpeg \
    python3 \
    python3-venv \
    python3-pip \
    python3-dev \
    gcc \
    snapd \
    procps \
    libx11-6 \
    libxcb1 \
    libxau6 \
    libxdmcp6 \
    libbsd0 \
    libmd0

if [ ! -x /usr/games/gamescope ]; then
    echo "Пакет gamescope встал, но нет /usr/games/gamescope" >&2
    exit 1
fi
if ! /usr/games/gamescope --help 2>&1 | grep -q -- '--force-grab-cursor'; then
    echo "Этот gamescope слишком старый: нет --force-grab-cursor." >&2
    echo "Нужен пакет 3.16+ из ppa:3v1n0/gamescope." >&2
    exit 1
fi
if ! ldconfig -p 2>/dev/null | grep -q 'libei.so.1'; then
    echo "Нет libei.so.1. Пакет libei1 не отдаёт библиотеку." >&2
    exit 1
fi

echo "== Steam (snap) =="
if ! snap list steam >/dev/null 2>&1; then
    sudo snap install steam
fi

echo "== виртуальная мышь uinput (старый ввод и коды клавиш) =="
if ! id -nG "$USER" | tr ' ' '\n' | grep -qx input; then
    sudo usermod -aG input "$USER"
    NEED_RELOGIN=1
fi
echo 'KERNEL=="uinput", GROUP="input", MODE="0660", OPTIONS+="static_node=uinput"' \
    | sudo tee /etc/udev/rules.d/99-uinput.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo modprobe uinput || true

echo "== чтение памяти своего CS2 =="
echo 'kernel.yama.ptrace_scope = 0' | sudo tee /etc/sysctl.d/99-ptrace.conf >/dev/null
sudo sysctl --system >/dev/null

echo "== Python =="
if [ ! -d "$ROOT/.venv" ]; then
    python3 -m venv "$ROOT/.venv"
fi
"$ROOT/.venv/bin/pip" install --upgrade pip
"$ROOT/.venv/bin/pip" install -r "$ROOT/python/requirements.txt"

echo
echo "Готово."
echo "  gamescope: $(/usr/games/gamescope --version 2>/dev/null || echo /usr/games/gamescope)"
echo "  libei:     $(ldconfig -p | awk '/libei.so.1 /{print $1; exit}')"
echo "  python:    $ROOT/.venv/bin/python"
if [ "${NEED_RELOGIN:-0}" -eq 1 ]; then
    echo
    echo "Выйди из сессии и зайди снова: пользователя добавили в группу input."
fi
echo
echo "Дальше, уже в сессии GNOME:"
echo "  cd \"$ROOT\""
echo "  source .venv/bin/activate"
echo "  python3 python/tools/run_dm.py"
echo "Игру отдельно не запускай. Если CS2 уже открыта обычным окном, закрой её:"
echo "бот не убивает такое окно и не поднимает вторую копию."
