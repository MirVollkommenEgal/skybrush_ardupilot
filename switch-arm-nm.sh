#!/usr/bin/env bash
set -euo pipefail

TARGET_LINK="/opt/homebrew/bin/arm-none-eabi-nm"
V14_BIN="/Applications/ArmGNUToolchain/14.3.rel1/arm-none-eabi/bin/arm-none-eabi-nm"
V10_BIN="/opt/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-nm"

usage() {
  cat <<USAGE
Verwendung: ${0##*/} <14|10|toggle|status>
  14      -> Link auf die v14 Toolchain setzen
  10      -> Link auf die v10 Toolchain setzen
  toggle  -> zwischen v14 und v10 umschalten
  status  -> aktuelles Target anzeigen
USAGE
}

ensure_link() {
  if [[ ! -L "$TARGET_LINK" ]]; then
    echo "Fehler: $TARGET_LINK ist kein Symlink." >&2
    exit 1
  fi
}

set_version() {
  local target="$1"
  if [[ ! -x "$target" ]]; then
    echo "Fehler: $target existiert nicht oder ist nicht ausführbar." >&2
    exit 1
  fi
  ln -sf "$target" "$TARGET_LINK"
  echo "arm-none-eabi-nm -> $target"
}

current_target() {
  if [[ -L "$TARGET_LINK" ]]; then
    readlink "$TARGET_LINK"
  else
    echo "unbekannt"
  fi
}

main() {
  local action="${1:-}" current
  if [[ -z "$action" ]]; then
    usage
    exit 1
  fi

  case "$action" in
    14|v14)
      set_version "$V14_BIN"
      ;;
    10|v10)
      set_version "$V10_BIN"
      ;;
    toggle)
      ensure_link
      current=$(current_target)
      if [[ "$current" == "$V14_BIN" ]]; then
        set_version "$V10_BIN"
      else
        set_version "$V14_BIN"
      fi
      ;;
    status)
      ensure_link
      echo "aktuell: $(current_target)"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
