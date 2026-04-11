#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-$(git rev-parse --show-toplevel)}"
APPS_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
ICON_PATH="$ROOT/doc/static/cu29.png"

if [[ ! -f "$ICON_PATH" ]]; then
  echo "missing icon asset: $ICON_PATH" >&2
  exit 1
fi

mkdir -p "$APPS_DIR"

write_entry() {
  local app_id="$1"
  local display_name="$2"

  cat >"$APPS_DIR/$app_id.desktop" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=$display_name
Exec=/bin/true
Icon=$ICON_PATH
Terminal=false
NoDisplay=true
StartupNotify=false
StartupWMClass=$app_id
EOF
}

write_entry "io.github.copper-project.balancebot-sim" "Copper BalanceBot Sim"
write_entry "io.github.copper-project.balancebot-bevymon" "Copper BalanceBot BevyMon"
write_entry "io.github.copper-project.flight-controller-sim" "Copper Flight Controller Sim"
write_entry "io.github.copper-project.flight-controller-bevymon" "Copper Flight Controller BevyMon"
write_entry "io.github.copper-project.bevymon-demo" "Copper BevyMon Demo"

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "$APPS_DIR" >/dev/null 2>&1 || true
fi

echo "Installed Copper sim desktop entries to $APPS_DIR"
