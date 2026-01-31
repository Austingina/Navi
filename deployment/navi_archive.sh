#!/bin/bash
set -e
set -u
# Try to enable pipefail, but continue if not supported
if (set -o pipefail 2>/dev/null); then
  set -o pipefail
fi

# Navi CI archive script
# Packages the following directories from Navi workspace:
#   - Navi/maps
#   - Navi/NaviAgent
#   - Navi/ros2_ws/install
#   - Navi/scripts
# The archive contains the Navi/ directory, so it can be extracted directly
# in the home directory: tar -xzf archive.tar.gz
#
# Usage: ./navi_archive.sh <arch> [version]
#   arch     – architecture identifier (required): arm or x86
#   version  – version number (optional): v0.0.1 format
#
# Examples:
#   ./navi_archive.sh arm                    -> navi_ubuntu_arm_20260127111125.tar.gz
#   ./navi_archive.sh arm v0.0.1             -> navi_ubuntu_arm_v0.0.1.tar.gz
#   ./navi_archive.sh x86 v1.2.3             -> navi_ubuntu_x86_v1.2.3.tar.gz
#
# Environment variables:
#   NAVI_WORKSPACE_DIR  – path to Navi workspace (defaults to ~/Navi)
#   NAVI_ARCHIVE_OUTPUT_DIR  – output directory (defaults to script directory)

# Parse arguments
if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <arch> [version]"
  echo "  arch     – architecture identifier (required): arm or x86"
  echo "  version  – version number (optional): v0.0.1 format"
  echo ""
  echo "Examples:"
  echo "  $0 arm                    -> navi_ubuntu_arm_20260127111125.tar.gz"
  echo "  $0 arm v0.0.1             -> navi_ubuntu_arm_v0.0.1.tar.gz"
  exit 1
fi

ARCH="$1"
VERSION="${2:-}"

# Validate architecture
if [[ "${ARCH}" != "arm" && "${ARCH}" != "x86" ]]; then
  echo "ERROR: Invalid architecture '${ARCH}'. Must be 'arm' or 'x86'"
  exit 1
fi

# Validate version format if provided
if [[ -n "${VERSION}" ]]; then
  if [[ ! "${VERSION}" =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "ERROR: Invalid version format '${VERSION}'. Must be in format v0.0.1"
    exit 1
  fi
fi

NAVI_WORKSPACE_DIR="${NAVI_WORKSPACE_DIR:-$HOME/Navi}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${NAVI_ARCHIVE_OUTPUT_DIR:-${SCRIPT_DIR}}"

# Generate timestamp (used for both archive name and archive_time)
TIMESTAMP=$(date +%Y%m%d%H%M%S)

# Generate archive name based on version
if [[ -n "${VERSION}" ]]; then
  NAVI_ARCHIVE_NAME="navi_ubuntu_${ARCH}_${VERSION}.tar.gz"
else
  NAVI_ARCHIVE_NAME="navi_ubuntu_${ARCH}_${TIMESTAMP}.tar.gz"
fi

echo "[navi_archive] Architecture: ${ARCH}"
if [[ -n "${VERSION}" ]]; then
  echo "[navi_archive] Version: ${VERSION}"
else
  echo "[navi_archive] Version: (timestamp-based)"
fi
echo "[navi_archive] Workspace: ${NAVI_WORKSPACE_DIR}"
echo "[navi_archive] Output archive: ${OUTPUT_DIR}/${NAVI_ARCHIVE_NAME}"

# Check if workspace exists
if [[ ! -d "${NAVI_WORKSPACE_DIR}" ]]; then
  echo "[navi_archive] ERROR: Workspace directory does not exist: ${NAVI_WORKSPACE_DIR}"
  exit 1
fi

# Check if required directories exist
REQUIRED_DIRS=(
  "${NAVI_WORKSPACE_DIR}/maps"
  "${NAVI_WORKSPACE_DIR}/NaviAgent"
  "${NAVI_WORKSPACE_DIR}/ros2_ws/install"
  "${NAVI_WORKSPACE_DIR}/scripts"
)

MISSING_DIRS=()
for dir in "${REQUIRED_DIRS[@]}"; do
  if [[ ! -d "${dir}" ]]; then
    MISSING_DIRS+=("${dir}")
  fi
done

if [[ ${#MISSING_DIRS[@]} -gt 0 ]]; then
  echo "[navi_archive] WARNING: Some required directories are missing:"
  for dir in "${MISSING_DIRS[@]}"; do
    echo "  - ${dir}"
  done
  echo "[navi_archive] Continuing anyway..."
fi

# Create output directory if it doesn't exist
mkdir -p "${OUTPUT_DIR}"

# Remove existing archive packages in the same directory
for _f in "${OUTPUT_DIR}"/navi_ubuntu_*.tar.gz; do
  if [[ -f "${_f}" ]]; then
    rm -f "${_f}"
    echo "[navi_archive] Removed existing: ${_f}"
  fi
done

# Generate version information
# TIMESTAMP is already generated above (format: YYYYMMDDHHMMSS)
if [[ -n "${VERSION}" ]]; then
  VERSION_STR="${VERSION}"
else
  VERSION_STR="${TIMESTAMP}"
fi

# Archive time uses the same format and value as TIMESTAMP
ARCHIVE_TIME="${TIMESTAMP}"

# Create version.txt file in Navi directory
VERSION_FILE="${NAVI_WORKSPACE_DIR}/version.txt"
echo "[navi_archive] Creating version.txt file..."
cat > "${VERSION_FILE}" <<EOF
version:${VERSION_STR}
archive_time:${ARCHIVE_TIME}
archive_name:${NAVI_ARCHIVE_NAME}
EOF
echo "[navi_archive] Version file created: ${VERSION_FILE}"

# Change to parent directory to include Navi/ in the archive
NAVI_PARENT_DIR="$(dirname "${NAVI_WORKSPACE_DIR}")"
NAVI_DIR_NAME="$(basename "${NAVI_WORKSPACE_DIR}")"
cd "${NAVI_PARENT_DIR}"

# Create archive with Navi/ directory included
echo "[navi_archive] Creating archive..."
tar -czf "${OUTPUT_DIR}/${NAVI_ARCHIVE_NAME}" \
  --exclude='.git' \
  --exclude='*.pyc' \
  --exclude='__pycache__' \
  --exclude='*.log' \
  "${NAVI_DIR_NAME}/maps/" \
  "${NAVI_DIR_NAME}/NaviAgent/" \
  "${NAVI_DIR_NAME}/ros2_ws/install/" \
  "${NAVI_DIR_NAME}/scripts/" \
  "${NAVI_DIR_NAME}/version.txt"

# Verify archive was created
if [[ -f "${OUTPUT_DIR}/${NAVI_ARCHIVE_NAME}" ]]; then
  echo "[navi_archive] Archive created successfully: ${OUTPUT_DIR}/${NAVI_ARCHIVE_NAME}"
  du -h "${OUTPUT_DIR}/${NAVI_ARCHIVE_NAME}"
else
  echo "[navi_archive] ERROR: Failed to create archive"
  exit 1
fi

echo "[navi_archive] Archive finished."
