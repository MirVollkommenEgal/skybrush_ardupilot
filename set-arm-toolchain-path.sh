#!/usr/bin/env bash
set -euo pipefail

TARGET_EXPORT='export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:/opt/homebrew/bin:$PATH'
FILES=("$HOME/.zprofile" "$HOME/.zshrc")

insert_line() {
  local file="$1"
  local tmp

  if [[ -f "$file" ]] && grep -Fxq "$TARGET_EXPORT" "$file"; then
    echo "\"$file\" enthält bereits den gewünschten PATH." 
    return
  fi

  tmp=$(mktemp)
  printf '%s\n\n' "$TARGET_EXPORT" > "$tmp"
  [[ -f "$file" ]] && cat "$file" >> "$tmp"
  mv "$tmp" "$file"
  echo "PATH-Zeile an den Anfang von $file gesetzt."
}

main() {
  for file in "${FILES[@]}"; do
    insert_line "$file"
  done
  echo "Fertig. Starte eine neue Shell mit \"exec zsh -l\" oder öffne ein neues Terminal."
}

main "$@"
