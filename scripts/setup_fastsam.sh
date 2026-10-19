#!/bin/bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MICROMAMBA_BIN="${MICROMAMBA_BIN:-${HOME}/.local/bin/micromamba}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-${HOME}/.micromamba}"
ENV_NAME="${ENV_NAME:-fastsam}"
PROXY_URL="${PROXY_URL:-http://192.168.0.81:7890}"
RKNN_WHEEL_DIR="${RKNN_WHEEL_DIR:-${HOME}/rknn-wheels}"
TOOLKIT_WHEEL="${RKNN_WHEEL_DIR}/rknn_toolkit2-2.3.2-cp39-cp39-manylinux_2_17_aarch64.manylinux2014_aarch64.whl"
LITE_WHEEL="${RKNN_WHEEL_DIR}/rknn_toolkit_lite2-2.3.2-cp39-cp39-manylinux_2_17_aarch64.manylinux2014_aarch64.whl"

if [ ! -x "${MICROMAMBA_BIN}" ]; then
  echo "micromamba not found at ${MICROMAMBA_BIN}" >&2
  exit 1
fi

if [ ! -f "${TOOLKIT_WHEEL}" ] || [ ! -f "${LITE_WHEEL}" ]; then
  echo "RKNN 2.3.2 CPython 3.9 ARM64 wheels are required in ${RKNN_WHEEL_DIR}" >&2
  exit 1
fi

if ! "${MICROMAMBA_BIN}" env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  HTTP_PROXY="${PROXY_URL}" HTTPS_PROXY="${PROXY_URL}" \
    "${MICROMAMBA_BIN}" create -y -n "${ENV_NAME}" -c conda-forge python=3.9 pip
fi

HTTP_PROXY="${PROXY_URL}" HTTPS_PROXY="${PROXY_URL}" \
  "${MICROMAMBA_BIN}" run -n "${ENV_NAME}" python -m pip install \
  --proxy "${PROXY_URL}" \
  --index-url https://pypi.org/simple \
  --timeout 90 \
  --retries 10 \
  -r "${WORKSPACE_DIR}/src/perception/perception_bringup/config/fastsam-requirements.txt" \
  "${TOOLKIT_WHEEL}" \
  "${LITE_WHEEL}"
