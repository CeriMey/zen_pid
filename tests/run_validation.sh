#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}"
OUT_BIN="${BUILD_DIR}/pid_validation"

g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/.." "${SCRIPT_DIR}/pid_validation.cpp" -o "${OUT_BIN}"

ARGS=("$@")
if [[ " ${ARGS[*]} " != *" --csv "* ]]; then
  DEFAULT_CSV="${SCRIPT_DIR}/data/latest_run.csv"
  mkdir -p "$(dirname "${DEFAULT_CSV}")"
  echo "[run_validation] --csv non spécifié, enregistrement par défaut dans ${DEFAULT_CSV}" >&2
  ARGS+=(--csv "${DEFAULT_CSV}")
fi

"${OUT_BIN}" "${ARGS[@]}"
