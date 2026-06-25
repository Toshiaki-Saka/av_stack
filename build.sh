#!/usr/bin/env bash
# One-command build + self-check for Linux / macOS.
# Usage:  ./build.sh
set -euo pipefail

echo "=== av-stack build ==="

# ── dependencies ──────────────────────────────────────────────────────────────
pip install pybind11 numpy matplotlib pytest --quiet

# ── cmake configure ───────────────────────────────────────────────────────────
cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"

# ── build ─────────────────────────────────────────────────────────────────────
cmake --build build -j"$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)"

# ── C++ self-check ────────────────────────────────────────────────────────────
echo ""
echo "=== C++ control test ==="
./build/control_test

# ── Python test suite ─────────────────────────────────────────────────────────
echo ""
echo "=== Python tests ==="
pytest tests/ -v

echo ""
echo "=== Build complete. ==="
echo "Run the animation demo:  python python/animation_demo.py"
