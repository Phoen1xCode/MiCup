#!/usr/bin/env bash
# Clean Python cache directories: __pycache__, .pytest_cache, *.pyc, *.pyo
# Usage: ./clean_cache.sh [path]  (default: current directory)

TARGET="${1:-.}"

echo "[INFO] Scanning ${TARGET} for cache files"

# Count
COUNT_CACHE=$(find "$TARGET" -type d -name "__pycache__" 2>/dev/null | wc -l | tr -d ' ')
COUNT_PYTEST=$(find "$TARGET" -type d -name ".pytest_cache" 2>/dev/null | wc -l | tr -d ' ')
COUNT_PYC=$(find "$TARGET" -type f \( -name "*.pyc" -o -name "*.pyo" \) 2>/dev/null | wc -l | tr -d ' ')

echo "[INFO] __pycache__   dirs: ${COUNT_CACHE}"
echo "[INFO] .pytest_cache dirs: ${COUNT_PYTEST}"
echo "[INFO] .pyc/.pyo     files: ${COUNT_PYC}"

# Delete
find "$TARGET" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
find "$TARGET" -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null
find "$TARGET" -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete 2>/dev/null

echo "[INFO] Python cache cleanup complete"
