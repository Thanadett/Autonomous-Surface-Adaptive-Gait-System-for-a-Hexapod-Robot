#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(dirname -- "$script_dir")"
cd -- "$workspace_dir"
# Same interpreter pin and build/install selection as build.sh and
# sim_smoke_test.sh: use build_final1/ + install_final1/ once they exist.
export PATH="/usr/bin:$PATH"
build_base="${COLCON_BUILD_BASE:-build}"
install_base="${COLCON_INSTALL_BASE:-install}"
if [[ -z "${COLCON_BUILD_BASE:-}" && -d build_final1 ]]; then
  build_base="build_final1"
  install_base="install_final1"
fi
colcon test --build-base "$build_base" --install-base "$install_base" --event-handlers console_direct+
colcon test-result --test-result-base "$build_base" --verbose
