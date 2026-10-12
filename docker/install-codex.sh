#!/usr/bin/env bash
set -euo pipefail

# CLI 和登录状态一起保存在 codex_data 卷中，重建容器后继续使用。
installer=$(mktemp)
trap 'rm -f "$installer"' EXIT
curl -fsSL https://chatgpt.com/codex/install.sh -o "$installer"
CODEX_INSTALL_DIR=/root/.codex/bin CODEX_NON_INTERACTIVE=true \
  sh "$installer" --release "${1:-0.151.0}"

# Docker 默认限制了 bubblewrap 所需的命名空间，使用 Landlock 保留沙箱隔离。
# 仅初始化配置，升级 CLI 时保留用户设置。
if [[ ! -f /root/.codex/config.toml ]]; then
  cat > /root/.codex/config.toml <<'EOF'
cli_auth_credentials_store = "file"

[features]
use_legacy_landlock = true
EOF
fi
