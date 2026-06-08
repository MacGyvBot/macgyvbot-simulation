#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ISAAC_SIM_PYTHON="${ISAAC_SIM_PYTHON:-}"

if [[ -z "${ISAAC_SIM_PYTHON}" ]]; then
  for candidate in \
    "/home/ssu/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh" \
    "/home/ssu/dev_ws/isaac_sim/isaacsim/source/scripts/python/linux-x86_64/python.sh" \
    "${HOME}/.local/share/ov/pkg/isaac-sim-4.5.0/python.sh" \
    "${HOME}/.local/share/ov/pkg/isaac-sim-5.1.0/python.sh"; do
    if [[ -x "${candidate}" ]]; then
      ISAAC_SIM_PYTHON="${candidate}"
      break
    fi
  done
fi

if [[ -z "${ISAAC_SIM_PYTHON}" || ! -x "${ISAAC_SIM_PYTHON}" ]]; then
  echo "Isaac Sim python.sh를 찾지 못했습니다." >&2
  echo "ISAAC_SIM_PYTHON=/path/to/python.sh ${0} [render options]" >&2
  exit 1
fi

cd "${REPO_ROOT}"
exec "${ISAAC_SIM_PYTHON}" scripts/isaacsim/render_macgvbot_usd.py "$@"
