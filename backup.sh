#!/usr/bin/env bash
# ==============================================================================
# Dual-Axis Pan-Tilt Controller Automated Backup Utility
# Archives source code, calibrated firmware, calibration tools, and web dashboard.
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BACKUP_DIR="${SCRIPT_DIR}/backups"
mkdir -p "$BACKUP_DIR"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
ARCHIVE_NAME="anzym_pan_tilt_backup_${TIMESTAMP}.tar.gz"
ARCHIVE_PATH="${BACKUP_DIR}/${ARCHIVE_NAME}"

echo "=========================================================="
echo " Starting Pan-Tilt Controller Backup: ${TIMESTAMP}"
echo "=========================================================="

echo "--> Archiving project repository..."
tar -czf "$ARCHIVE_PATH" \
    --exclude='.git' \
    --exclude='node_modules' \
    --exclude='build' \
    --exclude='build_*' \
    --exclude='backups' \
    --exclude='__pycache__' \
    --exclude='*.o' \
    --exclude='*.hex' \
    --exclude='*.bin' \
    --exclude='*.elf' \
    -C "$SCRIPT_DIR" .

SIZE=$(du -h "$ARCHIVE_PATH" | cut -f1)
SHA=$(sha256sum "$ARCHIVE_PATH" | cut -d' ' -f1)

echo "=========================================================="
echo " Backup Completed Successfully!"
echo " Archive:  ${ARCHIVE_PATH}"
echo " Size:     ${SIZE}"
echo " SHA-256:  ${SHA}"
echo "=========================================================="
