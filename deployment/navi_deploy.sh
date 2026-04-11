#!/bin/bash
set -e
set -u
# Try to enable pipefail, but continue if not supported
if (set -o pipefail 2>/dev/null); then
  set -o pipefail
fi

# Navi CI deploy script
# Deploys the archive created by navi_archive.sh to the target system.
# Usage: ./navi_deploy.sh <archive_file>
#   archive_file  – path to the archive file (tar.gz) to deploy
# Environment variables:
#   NAVI_DEPLOY_DIR  – target deployment directory (defaults to ~/Navi)
#   NAVI_BACKUP_DIR  – backup directory for existing deployment (defaults to ~/Navi.backup)

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <archive_file>"
  echo "  archive_file  – path to the archive file (tar.gz) to deploy"
  exit 1
fi

ARCHIVE_FILE_INPUT="$1"
NAVI_DEPLOY_DIR="${NAVI_DEPLOY_DIR:-$HOME/Navi}"
NAVI_BACKUP_DIR="${NAVI_BACKUP_DIR:-$HOME/Navi.backup}"

# Convert archive file path to absolute path to avoid issues when changing directories
if [[ "${ARCHIVE_FILE_INPUT}" = /* ]]; then
  # Already absolute path
  ARCHIVE_FILE="${ARCHIVE_FILE_INPUT}"
else
  # Relative path, convert to absolute
  ARCHIVE_FILE="$(cd "$(dirname "${ARCHIVE_FILE_INPUT}")" && pwd)/$(basename "${ARCHIVE_FILE_INPUT}")"
fi

echo "[navi_deploy] Archive file: ${ARCHIVE_FILE}"
echo "[navi_deploy] Target directory: ${NAVI_DEPLOY_DIR}"

# Check if archive file exists
if [[ ! -f "${ARCHIVE_FILE}" ]]; then
  echo "[navi_deploy] ERROR: Archive file does not exist: ${ARCHIVE_FILE}"
  exit 1
fi

# Check if archive file is readable
if [[ ! -r "${ARCHIVE_FILE}" ]]; then
  echo "[navi_deploy] ERROR: Archive file is not readable: ${ARCHIVE_FILE}"
  exit 1
fi

# Verify archive format
if ! tar -tzf "${ARCHIVE_FILE}" >/dev/null 2>&1; then
  echo "[navi_deploy] ERROR: Invalid archive file or corrupted: ${ARCHIVE_FILE}"
  exit 1
fi

# Check if target directory already exists
if [[ -d "${NAVI_DEPLOY_DIR}" ]]; then
  echo "[navi_deploy] WARNING: Target directory already exists: ${NAVI_DEPLOY_DIR}"
  
  # Read version from version.txt if it exists
  VERSION_FILE="${NAVI_DEPLOY_DIR}/version.txt"
  BACKUP_SUFFIX=""
  
  if [[ -f "${VERSION_FILE}" ]]; then
    # Try to extract version field
    VERSION_VALUE=$(grep -E "^version:" "${VERSION_FILE}" | cut -d':' -f2 | tr -d '[:space:]')
    if [[ -n "${VERSION_VALUE}" ]]; then
      BACKUP_SUFFIX="${VERSION_VALUE}"
      echo "[navi_deploy] Found version in version.txt: ${VERSION_VALUE}"
    fi
  fi
  
  # If no version found, use timestamp
  if [[ -z "${BACKUP_SUFFIX}" ]]; then
    BACKUP_TIMESTAMP=$(date +%Y%m%d%H%M%S)
    BACKUP_SUFFIX="${BACKUP_TIMESTAMP}"
    echo "[navi_deploy] No version found, using timestamp: ${BACKUP_TIMESTAMP}"
  fi
  
  # Generate base backup path
  BASE_BACKUP_PATH="${NAVI_BACKUP_DIR}.${BACKUP_SUFFIX}"
  BACKUP_PATH="${BASE_BACKUP_PATH}"
  
  # Check if backup path already exists and find next available suffix
  # First backup: Navi.backup.v0.0.1, second: Navi.backup.v0.0.1-1, third: Navi.backup.v0.0.1-2, ...
  if [[ -d "${BACKUP_PATH}" ]] || [[ -e "${BACKUP_PATH}" ]]; then
    echo "[navi_deploy] Backup path ${BACKUP_PATH} already exists, finding next available suffix..."
    COUNTER=1
    while [[ -d "${BASE_BACKUP_PATH}-${COUNTER}" ]] || [[ -e "${BASE_BACKUP_PATH}-${COUNTER}" ]]; do
      COUNTER=$((COUNTER + 1))
    done
    BACKUP_PATH="${BASE_BACKUP_PATH}-${COUNTER}"
    echo "[navi_deploy] Using backup path: ${BACKUP_PATH}"
  fi
  
  # Clean up old backups if total backup count exceeds 10 (before creating new backup)
  # This applies to ALL backups across all versions, not just the current version
  echo "[navi_deploy] Checking for old backups to clean up..."
  BACKUP_BASE_PATTERN=$(basename "${NAVI_BACKUP_DIR}")
  BACKUP_DIR=$(dirname "${NAVI_BACKUP_DIR}")
  
  # Find all backups matching the pattern (e.g., Navi.backup.v0.0.1, Navi.backup.v0.0.2, Navi.backup.20260127, etc.)
  BACKUP_COUNT=$(find "${BACKUP_DIR}" -maxdepth 1 -type d -name "${BACKUP_BASE_PATTERN}.*" 2>/dev/null | wc -l)
  
  if [[ ${BACKUP_COUNT} -ge 10 ]]; then
    echo "[navi_deploy] Found ${BACKUP_COUNT} backups (all versions), cleaning up oldest (keeping max 10)..."
    # Find all backups, sort by modification time, delete oldest
    # Keep only the 9 most recent, delete the rest
    DELETE_COUNT=$((BACKUP_COUNT - 9))
    find "${BACKUP_DIR}" -maxdepth 1 -type d -name "${BACKUP_BASE_PATTERN}.*" -printf '%T@ %p\n' 2>/dev/null | \
      sort -n | \
      head -n "${DELETE_COUNT}" | \
      while read -r timestamp path; do
        echo "[navi_deploy] Removing old backup: ${path}"
        rm -rf "${path}"
      done
  fi
  
  echo "[navi_deploy] Creating backup: ${BACKUP_PATH}"
  mv "${NAVI_DEPLOY_DIR}" "${BACKUP_PATH}"
  echo "[navi_deploy] Backup created: ${BACKUP_PATH}"
fi

# Check archive top-level structure to avoid duplicate Navi directory
echo "[navi_deploy] Checking archive structure..."
# Get all unique top-level directories in the archive
TOP_LEVEL_DIRS=$(tar -tzf "${ARCHIVE_FILE}" | cut -d'/' -f1 | sort -u)
HAS_NAVI_DIR=false

# Check if any top-level directory is "Navi"
while IFS= read -r dir; do
  if [[ "${dir}" == "Navi" ]]; then
    HAS_NAVI_DIR=true
    break
  fi
done <<< "${TOP_LEVEL_DIRS}"

echo "[navi_deploy] Top-level directories in archive: $(echo ${TOP_LEVEL_DIRS} | tr '\n' ' ')"

# Determine extraction directory based on archive structure
if [[ "${HAS_NAVI_DIR}" == "true" ]]; then
  # Archive contains Navi/ directory, extract to parent directory
  EXTRACT_DIR="$(dirname "${NAVI_DEPLOY_DIR}")"
  echo "[navi_deploy] Archive contains Navi/ directory, extracting to: ${EXTRACT_DIR}"
  mkdir -p "${EXTRACT_DIR}"
  cd "${EXTRACT_DIR}"
else
  # Archive contains maps/, NaviSkill/, etc. directly, extract to target directory
  echo "[navi_deploy] Archive contains directories directly, extracting to: ${NAVI_DEPLOY_DIR}"
  mkdir -p "${NAVI_DEPLOY_DIR}"
  cd "${NAVI_DEPLOY_DIR}"
fi

# Extract archive
echo "[navi_deploy] Extracting archive..."
tar -xzf "${ARCHIVE_FILE}"

# Verify extraction (final location is always NAVI_DEPLOY_DIR)
echo "[navi_deploy] Verifying deployment..."
if [[ -d "${NAVI_DEPLOY_DIR}/maps" ]] || \
   [[ -d "${NAVI_DEPLOY_DIR}/NaviSkill" ]] || \
   [[ -d "${NAVI_DEPLOY_DIR}/ros2_ws/install" ]] || \
   [[ -d "${NAVI_DEPLOY_DIR}/scripts" ]]; then
  echo "[navi_deploy] Deployment successful!"
  echo "[navi_deploy] Deployed to: ${NAVI_DEPLOY_DIR}"
  echo "[navi_deploy] Deployed directories:"
  [[ -d "${NAVI_DEPLOY_DIR}/maps" ]] && echo "  - maps/"
  [[ -d "${NAVI_DEPLOY_DIR}/NaviSkill" ]] && echo "  - NaviSkill/"
  [[ -d "${NAVI_DEPLOY_DIR}/ros2_ws/install" ]] && echo "  - ros2_ws/install/"
  [[ -d "${NAVI_DEPLOY_DIR}/scripts" ]] && echo "  - scripts/"
else
  echo "[navi_deploy] WARNING: Expected directories not found after extraction"
  echo "[navi_deploy] Please verify the archive contents"
  echo "[navi_deploy] Expected location: ${NAVI_DEPLOY_DIR}"
fi

echo "[navi_deploy] Deployment finished."
