#!/usr/bin/env bash
# One-time setup: creates .venv and installs dependencies.
# On recent macOS, `pip install pybullet` fails to compile (missing C++ headers
# and an old zlib macro clashing with the new SDK). This script applies the fix.
set -e
cd "$(dirname "$0")"
python3 -m venv .venv
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q numpy matplotlib pytest

if .venv/bin/pip install -q pybullet 2>/dev/null; then
  echo "pybullet installed from pip"
elif [[ "$(uname)" == "Darwin" ]]; then
  echo "Standard pybullet build failed; building a patched copy for macOS (takes a few minutes)..."
  TMP=$(mktemp -d)
  .venv/bin/pip download -q --no-deps --no-binary :all: pybullet -d "$TMP"
  tar xzf "$TMP"/pybullet-*.tar.gz -C "$TMP"
  SRC=$(ls -d "$TMP"/pybullet-*/)
  # remove the '#define fdopen(fd,mode) NULL' block that breaks the new macOS SDK headers
  sed -i '' '/^#ifndef fdopen$/,/^#endif$/d' "$SRC/examples/ThirdPartyLibs/zlib/zutil.h"
  SDK=$(xcrun --show-sdk-path)
  ARCHFLAGS="-arch $(uname -m)" MACOSX_DEPLOYMENT_TARGET=13.0 SDKROOT="$SDK" \
    CPLUS_INCLUDE_PATH="$SDK/usr/include/c++/v1" .venv/bin/pip install -q "$SRC"
  rm -rf "$TMP"
else
  echo "pybullet install failed" >&2; exit 1
fi
.venv/bin/python -c "import pybullet; print('setup OK')"
