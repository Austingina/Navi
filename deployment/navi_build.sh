#!/usr/bin/env bash
set -euo pipefail

# Navi CI build script
# Follows README steps 1–4 for the *编译机* (no GUI, no start_navi/start_rviz).
# Repo URL and branch are taken from environment variables:
#   NAVI_REPO_URL     – git URL of the repo (defaults to git@gitlab.autel.com:/avant/AI-Embodied/Mobility/fast_lio2.git)
#   NAVI_REPO_BRANCH  – branch name (defaults to "navi")
# SSH key for git is taken from:
#   NAVI_SSH_KEY      – path to the private key (defaults to ~/CI/id_yyf)
#
# After build, archive and upload to MinIO by default (set NAVI_DO_ARCHIVE=0 to skip)
#   NAVI_ARCH           – arch for archive (defaults to "arm")
#   NAVI_VERSION        – version for archive (optional, e.g. v0.0.1)
#   NAVI_MINIO_ENDPOINT – MinIO API endpoint (defaults to http://10.239.121.20:9000)
#   NAVI_MINIO_BUCKET   – bucket name (defaults to "physical")
#   NAVI_MINIO_PATH     – path in bucket (defaults to "Navi")
#   NAVI_MINIO_USER     – MinIO user (defaults to "minio")
#   NAVI_MINIO_PASSWORD – MinIO password (defaults to "minio123")

NAVI_REPO_URL="${NAVI_REPO_URL:-git@gitlab.autel.com:/avant/AI-Embodied/Mobility/fast_lio2.git}"
NAVI_REPO_BRANCH="${NAVI_REPO_BRANCH:-navi}"
NAVI_WORKSPACE_DIR="${NAVI_WORKSPACE_DIR:-$HOME/Navi}"
NAVI_SSH_KEY_DEFAULT="$HOME/CI/id_yyf"
NAVI_SSH_KEY="${NAVI_SSH_KEY:-$NAVI_SSH_KEY_DEFAULT}"

export GIT_SSH_COMMAND="ssh -i ${NAVI_SSH_KEY}"

echo "[navi_build] Using repo: ${NAVI_REPO_URL} (branch: ${NAVI_REPO_BRANCH})"
echo "[navi_build] Workspace: ${NAVI_WORKSPACE_DIR}"
echo "[navi_build] SSH key: ${NAVI_SSH_KEY}"

# Ensure workspace root exists
mkdir -p "${NAVI_WORKSPACE_DIR}"

if [[ ! -d "${NAVI_WORKSPACE_DIR}/.git" ]]; then
  echo "[navi_build] Cloning repository..."
  git clone -b "${NAVI_REPO_BRANCH}" "${NAVI_REPO_URL}" "${NAVI_WORKSPACE_DIR}"
else
  echo "[navi_build] Repository already exists at ${NAVI_WORKSPACE_DIR}, skipping clone."
fi

cd "${NAVI_WORKSPACE_DIR}"

echo "[navi_build] Step 2: build & install 3D BBS"
cd "${NAVI_WORKSPACE_DIR:?}/ros2_ws/src/3rdparty/3d_bbs"
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"

if [[ "${NAVI_SKIP_INSTALL:-0}" != "1" ]]; then
  echo "[navi_build] Running 'sudo make install' for 3D BBS"
  sudo make install
else
  echo "[navi_build] Skipping 'make install' for 3D BBS because NAVI_SKIP_INSTALL=1"
fi

echo "[navi_build] Step 3: build Livox-SDK2"
# Vendored in-tree under 3rdparty/; the old ros2_ws/src/Livox-SDK2 submodule
# path no longer exists.
cd "${NAVI_WORKSPACE_DIR}/ros2_ws/src/3rdparty/Livox-SDK2"
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"

if [[ "${NAVI_SKIP_INSTALL:-0}" != "1" ]]; then
  echo "[navi_build] Running 'sudo make install' for Livox-SDK2"
  sudo make install
else
  echo "[navi_build] Skipping 'make install' for Livox-SDK2 because NAVI_SKIP_INSTALL=1"
fi

echo "[navi_build] Step 4: colcon build (编译机, no --symlink-install)"
cd "${NAVI_WORKSPACE_DIR}/ros2_ws"

NAVI_ROS_DISTRO="${NAVI_ROS_DISTRO:-jazzy}"

if [[ -f "/opt/ros/${NAVI_ROS_DISTRO}/setup.bash" ]]; then
  # Load ROS 2 environment if available.
  # 注意：/opt/ros/<distro>/setup.bash 里会访问一些可能未定义的环境变量
  #（例如 AMENT_TRACE_SETUP_FILES），在 set -u 下会报“unbound variable”，
  #所以这里临时关闭 -u，再恢复。
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${NAVI_ROS_DISTRO}/setup.bash"
  set -u
else
  echo "[navi_build] WARNING: /opt/ros/${NAVI_ROS_DISTRO}/setup.bash not found. Make sure ROS 2 ${NAVI_ROS_DISTRO} is installed and sourced."
fi

echo "[navi_build] Importing source-only dependencies (ros2_numpy)"
# ros2_numpy is imported by elevation_mapping_cupy's node but is not released to
# apt on any distro, so it has to come from git. The manifest lives with
# elevation_mapping_cupy because that is the only consumer.
if [[ ! -d src/ros2_numpy ]]; then
  vcs import src < src/elevation_mapping_cupy/docker/src.repos
else
  echo "[navi_build] src/ros2_numpy already present, skipping import."
fi

echo "[navi_build] Resolving ROS dependencies with rosdep"
set +u
# cupy-cuda13x and simple-parsing are local keys from
# elevation_mapping_cupy/docker/extra_rosdeps.yaml, which is not registered with
# rosdep here; both come from the requirements files / venv instead.
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "cupy-cuda13x simple-parsing"
set -u

colcon build \
  --cmake-args -DROS_EDITION=ROS2 -DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE \
  --event-handlers console_direct+

echo "[navi_build] Build finished."

# ------------------------------------------------------------------------------
# Archive and upload to MinIO (optional, only when NAVI_DO_ARCHIVE=1)
# ------------------------------------------------------------------------------
if [[ "${NAVI_DO_ARCHIVE:-1}" == "1" ]]; then
  # Switch to CI dir (same level as workspace, e.g. ~/CI when NAVI_WORKSPACE_DIR=~/Navi)
  CI_SCRIPT_DIR="$(cd "$(dirname "${NAVI_WORKSPACE_DIR}")/CI" && pwd)"
  cd "${CI_SCRIPT_DIR}"

  NAVI_ARCH="${NAVI_ARCH:-arm}"
  NAVI_VERSION="${NAVI_VERSION:-}"
  export NAVI_WORKSPACE_DIR
  export NAVI_ARCHIVE_OUTPUT_DIR="${NAVI_ARCHIVE_OUTPUT_DIR:-${CI_SCRIPT_DIR}}"

  echo "[navi_build] Running archive script in ${CI_SCRIPT_DIR}..."
  if [[ -n "${NAVI_VERSION}" ]]; then
    bash ./navi_archive.sh "${NAVI_ARCH}" "${NAVI_VERSION}"
  else
    bash ./navi_archive.sh "${NAVI_ARCH}"
  fi
  _archive_exit=$?
  if [[ ${_archive_exit} -ne 0 ]]; then
    echo "[navi_build] ERROR: navi_archive.sh exited with ${_archive_exit}"
    exit ${_archive_exit}
  fi

  # 归档成功时只有一个包，用 glob 取匹配到的即可（无管道，避免 SIGPIPE/141）
  _archs=("${NAVI_ARCHIVE_OUTPUT_DIR}"/navi_ubuntu_${NAVI_ARCH}_*.tar.gz)
  ARCHIVE_FILE="${_archs[0]:-}"
  if [[ -z "${ARCHIVE_FILE}" || ! -f "${ARCHIVE_FILE}" ]]; then
    echo "[navi_build] ERROR: Archive file not found after navi_archive.sh"
    echo "[navi_build]   Looked in: ${NAVI_ARCHIVE_OUTPUT_DIR}"
    echo "[navi_build]   Pattern:   navi_ubuntu_${NAVI_ARCH}_*.tar.gz"
    exit 1
  fi
  echo "[navi_build] Archive to upload: ${ARCHIVE_FILE}"

  echo "[navi_build] Uploading to MinIO..."
  # MinIO config. Default target corresponds to http://10.239.121.20:9001/browser/physical/Navi
  NAVI_MINIO_ENDPOINT="${NAVI_MINIO_ENDPOINT:-http://10.239.121.20:9000}"
  NAVI_MINIO_ALIAS="${NAVI_MINIO_ALIAS:-navi_minio}"
  NAVI_MINIO_BUCKET="${NAVI_MINIO_BUCKET:-physical}"
  NAVI_MINIO_PATH="${NAVI_MINIO_PATH:-Navi}"
  NAVI_MINIO_USER="${NAVI_MINIO_USER:-minio}"
  NAVI_MINIO_PASSWORD="${NAVI_MINIO_PASSWORD:-minio123}"

  if ! command -v mc &>/dev/null; then
    echo "[navi_build] ERROR: mc (MinIO Client) not found. See: https://min.io/docs/minio/linux/reference/minio-mc.html"
    exit 1
  fi

  if ! mc alias set "${NAVI_MINIO_ALIAS}" "${NAVI_MINIO_ENDPOINT}" "${NAVI_MINIO_USER}" "${NAVI_MINIO_PASSWORD}"; then
    echo "[navi_build] ERROR: mc alias set failed (endpoint=${NAVI_MINIO_ENDPOINT})"
    exit 1
  fi
  if ! mc cp "${ARCHIVE_FILE}" "${NAVI_MINIO_ALIAS}/${NAVI_MINIO_BUCKET}/${NAVI_MINIO_PATH}/"; then
    echo "[navi_build] ERROR: mc cp failed (file=${ARCHIVE_FILE}, target=${NAVI_MINIO_ALIAS}/${NAVI_MINIO_BUCKET}/${NAVI_MINIO_PATH}/)"
    exit 1
  fi
  echo "[navi_build] Upload OK: ${ARCHIVE_FILE} -> ${NAVI_MINIO_ENDPOINT}/${NAVI_MINIO_BUCKET}/${NAVI_MINIO_PATH}/"
fi