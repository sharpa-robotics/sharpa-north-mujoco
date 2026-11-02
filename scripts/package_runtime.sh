#!/usr/bin/env bash
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONFIG_PATH="${PROJECT_ROOT}/sharpa_mujoco.conf"
MUJOCO_ROOT=""
MODEL_PATH=""
OUTPUT_DIR="${PROJECT_ROOT}/dist"
BUILD_DIR="${PROJECT_ROOT}/build-package-linux"
PACKAGE_NAME=""
JOBS="${JOBS:-$(nproc)}"
STAGE_PLATFORM=ON
PLATFORM_MODE=with

usage() {
    cat <<'EOF'
Usage: ./scripts/package_runtime.sh [options]

Build a self-contained Sharpa MuJoCo Linux runtime package. The package
contains the simulator, robot model, sensor resources, libmujoco, optional
Viewer/libglfw, configuration, licenses, and a top-level run.sh.

Options:
      --mujoco-root PATH   MuJoCo 3.8.1 root (default: mujoco_root in config)
      --without-platform   Package MuJoCo only using build-package-linux-no-platform
  -h, --help               Show this help

Examples:
  ./scripts/package_runtime.sh
  ./scripts/package_runtime.sh --without-platform
  ./scripts/package_runtime.sh --mujoco-root /opt/mujoco-3.8.1
EOF
}

absolute_path() {
    local path="$1"
    local base="${2:-$(pwd)}"
    if [[ "${path}" = /* ]]; then
        printf '%s\n' "${path}"
    else
        printf '%s/%s\n' "${base}" "${path}"
    fi
}

read_root_value() {
    local path="$1"
    local wanted="$2"
    awk -F= -v wanted="${wanted}" '
        /^[[:space:]]*($|#|\[)/ { next }
        {
            key=$1
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", key)
            if (key == wanted) {
                value=substr($0, index($0, "=") + 1)
                sub(/[[:space:]]*#.*/, "", value)
                gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
                gsub(/^[\047\"]|[\047\"]$/, "", value)
                print value
                exit
            }
        }
    ' "${path}"
}

read_section_value() {
    local path="$1"
    local wanted_section="$2"
    local wanted_key="$3"
    awk -F= -v wanted_section="${wanted_section}" -v wanted_key="${wanted_key}" '
        /^[[:space:]]*\[/ {
            section=$0
            gsub(/^[[:space:]]*\[|\][[:space:]]*$/, "", section)
            next
        }
        section == wanted_section {
            key=$1
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", key)
            if (key == wanted_key) {
                value=substr($0, index($0, "=") + 1)
                sub(/[[:space:]]*#.*/, "", value)
                gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
                gsub(/^[\047\"]|[\047\"]$/, "", value)
                print value
                exit
            }
        }
    ' "${path}"
}

fail() {
    echo "error: $*" >&2
    exit 1
}

validate_elf_arch() {
    local readelf_tool="$1"
    local path="$2"
    local expected_arch="$3"
    local machine
    machine="$(
        LC_ALL=C "${readelf_tool}" -h "${path}" |
            awk -F: '$1 ~ /^[[:space:]]*Machine[[:space:]]*$/ {
                value=$2
                gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
                print value
                exit
            }'
    )"
    [[ -n "${machine}" ]] || fail "could not read ELF architecture: ${path}"
    case "${expected_arch}:${machine}" in
        aarch64:AArch64|x86_64:*X86-64*) ;;
        *) fail "wrong ELF architecture for ${path}: expected ${expected_arch}, found ${machine}" ;;
    esac
}

while (($# > 0)); do
    case "$1" in
        --without-platform) STAGE_PLATFORM=OFF; PLATFORM_MODE=without; shift ;;
        --mujoco-root)
            (($# >= 2)) || fail "$1 requires a path"
            MUJOCO_ROOT="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            fail "unknown option: $1"
            ;;
    esac
done

[[ "$(uname -s)" == "Linux" ]] || fail "runtime packages can only be built on Linux"

case "$(uname -m)" in
    x86_64|amd64) HOST_ARCH=x86_64 ;;
    aarch64|arm64) HOST_ARCH=aarch64 ;;
    *) fail "unsupported build host architecture: $(uname -m)" ;;
esac

PACKAGE_ARCH="${HOST_ARCH}"
BUILD_VIEWER=ON
if [[ "${PLATFORM_MODE}" == without ]]; then
    BUILD_DIR="${BUILD_DIR}-no-platform"
fi

[[ "${JOBS}" =~ ^[1-9][0-9]*$ ]] || fail "JOBS must be a positive integer"

CONFIG_PATH="$(absolute_path "${CONFIG_PATH}")"
CONFIG_PATH="$(realpath -e -- "${CONFIG_PATH}")"
CONFIG_DIR="$(dirname -- "${CONFIG_PATH}")"

if [[ -z "${MUJOCO_ROOT}" ]]; then
    MUJOCO_ROOT="$(read_root_value "${CONFIG_PATH}" mujoco_root)"
fi
[[ -n "${MUJOCO_ROOT}" ]] || fail "MuJoCo root is not set; pass --mujoco-root"
MUJOCO_ROOT="$(absolute_path "${MUJOCO_ROOT}" "${CONFIG_DIR}")"
MUJOCO_ROOT="$(realpath -e -- "${MUJOCO_ROOT}")"

MODEL_PATH="$(read_section_value "${CONFIG_PATH}" simulation model)"
[[ -n "${MODEL_PATH}" ]] || fail "simulation.model is not set in ${CONFIG_PATH}"
MODEL_PATH="$(absolute_path "${MODEL_PATH}" "${CONFIG_DIR}")"
MODEL_PATH="$(realpath -e -- "${MODEL_PATH}")"
MODEL_DIR="$(dirname -- "${MODEL_PATH}")"

[[ -f "${MUJOCO_ROOT}/include/mujoco/mujoco.h" ]] ||
    fail "MuJoCo headers are missing under ${MUJOCO_ROOT}/include"
if [[ "${BUILD_VIEWER}" == ON ]]; then
    [[ -d "${MUJOCO_ROOT}/simulate" ]] ||
        fail "MuJoCo Viewer sources are missing under ${MUJOCO_ROOT}/simulate"
fi
[[ -f "${MODEL_PATH}" ]] || fail "model does not exist: ${MODEL_PATH}"

for command in cmake awk find readelf realpath tar xz; do
    command -v "${command}" >/dev/null 2>&1 || fail "required command is unavailable: ${command}"
done
command -v ldd >/dev/null 2>&1 || fail "required command is unavailable: ldd"

PROJECT_VERSION="$(
    awk '$1 == "VERSION" { print $2; exit }' "${PROJECT_ROOT}/CMakeLists.txt"
)"
[[ -n "${PROJECT_VERSION}" ]] || fail "could not read the project version"
PACKAGE_NAME="sharpa-mujoco-${PROJECT_VERSION}-linux-${PACKAGE_ARCH}"
if [[ "${PLATFORM_MODE}" == without ]]; then
    PACKAGE_NAME="${PACKAGE_NAME}-no-platform"
fi

MUJOCO_INPUT_RUNTIME=""
for candidate in \
    "${MUJOCO_ROOT}/lib/libmujoco.so.3.8.1" \
    "${MUJOCO_ROOT}/lib/libmujoco.so" \
    "${MUJOCO_ROOT}/bin/libmujoco.so.3.8.1" \
    "${MUJOCO_ROOT}/bin/libmujoco.so"; do
    if [[ -e "${candidate}" ]]; then
        MUJOCO_INPUT_RUNTIME="$(realpath -e -- "${candidate}")"
        break
    fi
done
[[ -n "${MUJOCO_INPUT_RUNTIME}" ]] ||
    fail "MuJoCo runtime library is missing under ${MUJOCO_ROOT}/lib"
validate_elf_arch readelf "${MUJOCO_INPUT_RUNTIME}" "${PACKAGE_ARCH}"

OUTPUT_DIR="$(absolute_path "${OUTPUT_DIR}")"
BUILD_DIR="$(absolute_path "${BUILD_DIR}")"
PACKAGE_DIR="${OUTPUT_DIR}/${PACKAGE_NAME}"
ARCHIVE_PATH="${OUTPUT_DIR}/${PACKAGE_NAME}.tar.xz"

mkdir -p -- "${OUTPUT_DIR}"
if [[ -e "${PACKAGE_DIR}" || -e "${ARCHIVE_PATH}" ]]; then
    echo "Replacing existing runtime package: ${PACKAGE_NAME}"
    rm -rf -- "${PACKAGE_DIR}"
    rm -f -- "${ARCHIVE_PATH}"
fi

echo "Configuring the Linux runtime build"
echo "  MuJoCo: ${MUJOCO_ROOT}"
echo "  model:  ${MODEL_PATH}"
echo "  build:  ${BUILD_DIR}"
echo "  output: ${PACKAGE_DIR}"
echo "  target: native (${PACKAGE_ARCH})"
echo "  viewer: ${BUILD_VIEWER}"
echo "  platform: ${STAGE_PLATFORM}"

CMAKE_OPTIONS=(
    -S "${PROJECT_ROOT}"
    -B "${BUILD_DIR}"
    -U 'MuJoCo_*'
    -U 'mujoco_*'
    -DCMAKE_BUILD_TYPE=MinSizeRel
    -DMUJOCO_ROOT="${MUJOCO_ROOT}"
    -DSHARPA_MUJOCO_BUILD_TESTS=OFF
    -DSHARPA_MUJOCO_BUILD_DEVICE_PLUGINS=ON
    -DSHARPA_MUJOCO_BUILD_SIMULATOR=ON
    -DSHARPA_MUJOCO_BUILD_EMBEDDED_SIMULATOR=OFF
    -DSHARPA_MUJOCO_BUILD_VIEWER="${BUILD_VIEWER}"
    -DSHARPA_MUJOCO_PORTABLE_RUNTIME=ON
    -DSHARPA_MUJOCO_STAGE_PLATFORM="${STAGE_PLATFORM}"
)

cmake "${CMAKE_OPTIONS[@]}"

cmake --build "${BUILD_DIR}" \
    --target sharpa_mujoco_simulator sharpa_mujoco_environment \
    --parallel "${JOBS}"

SIMULATOR_SOURCE="${BUILD_DIR}/apps/simulator/sharpa_mujoco_simulator"
ENVIRONMENT_SOURCE="${BUILD_DIR}/sharpa_mujoco_environment"
[[ -x "${SIMULATOR_SOURCE}" ]] || fail "built simulator is missing: ${SIMULATOR_SOURCE}"
[[ -x "${ENVIRONMENT_SOURCE}" ]] || fail "built environment checker is missing: ${ENVIRONMENT_SOURCE}"

MUJOCO_RUNTIME="$(
    awk -F= '
        $1 == "MuJoCo_RUNTIME_LIBRARY:FILEPATH" {
            print substr($0, index($0, "=") + 1)
            exit
        }
    ' "${BUILD_DIR}/CMakeCache.txt"
)"
[[ -f "${MUJOCO_RUNTIME}" ]] || fail "could not resolve the MuJoCo runtime library"
MUJOCO_RUNTIME="$(realpath -e -- "${MUJOCO_RUNTIME}")"
if [[ "${MUJOCO_RUNTIME}" != "${MUJOCO_ROOT}/"* ]]; then
    fail "CMake resolved MuJoCo outside --mujoco-root: ${MUJOCO_RUNTIME}"
fi
validate_elf_arch readelf "${MUJOCO_RUNTIME}" "${PACKAGE_ARCH}"
validate_elf_arch readelf "${SIMULATOR_SOURCE}" "${PACKAGE_ARCH}"
validate_elf_arch readelf "${ENVIRONMENT_SOURCE}" "${PACKAGE_ARCH}"

GLFW_SONAME=""
GLFW_RUNTIME=""
BUILD_LIBRARY_PATH="$(dirname -- "${MUJOCO_RUNTIME}")${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
BUILD_LDD_OUTPUT="$(LD_LIBRARY_PATH="${BUILD_LIBRARY_PATH}" ldd "${SIMULATOR_SOURCE}")"
MUJOCO_SONAME="$(
    printf '%s\n' "${BUILD_LDD_OUTPUT}" |
        awk '$1 ~ /^libmujoco\.so/ && $2 == "=>" { print $1; exit }'
)"

GLFW_SONAME="$(
    printf '%s\n' "${BUILD_LDD_OUTPUT}" |
        awk '$1 ~ /^libglfw\.so/ && $2 == "=>" { print $1; exit }'
)"
GLFW_RUNTIME="$(
    printf '%s\n' "${BUILD_LDD_OUTPUT}" |
        awk '$1 ~ /^libglfw\.so/ && $2 == "=>" { print $3; exit }'
)"
[[ -n "${GLFW_SONAME}" && -f "${GLFW_RUNTIME}" ]] ||
    fail "could not resolve the GLFW runtime library"
[[ -n "${MUJOCO_SONAME}" ]] || fail "the simulator does not link to libmujoco"

STAGING_DIR="$(mktemp -d "${OUTPUT_DIR}/.${PACKAGE_NAME}.staging.XXXXXX")"
cleanup() {
    rm -rf -- "${STAGING_DIR}"
}
trap cleanup EXIT

install -d \
    "${STAGING_DIR}/bin" \
    "${STAGING_DIR}/lib" \
    "${STAGING_DIR}/scripts" \
    "${STAGING_DIR}/licenses/mujoco"
if [[ "${BUILD_VIEWER}" == ON ]]; then
    install -d "${STAGING_DIR}/licenses/glfw"
fi

install -m 0755 "${SIMULATOR_SOURCE}" \
    "${STAGING_DIR}/bin/sharpa_mujoco_simulator"
install -m 0755 "${ENVIRONMENT_SOURCE}" \
    "${STAGING_DIR}/bin/sharpa_mujoco_environment"
if grep -q '^SHARPA_MUJOCO_STAGE_PLATFORM:BOOL=ON$' "${BUILD_DIR}/CMakeCache.txt"; then
    install -d "${STAGING_DIR}/lib/sharpa_mujoco/companions"
    cp -a -- "${BUILD_DIR}/sharpa_mujoco/companions/sharpa_platform" \
        "${STAGING_DIR}/lib/sharpa_mujoco/companions/"
fi
install -m 0755 "${SCRIPT_DIR}/runtime_run.sh" "${STAGING_DIR}/run.sh"
install -m 0755 "${SCRIPT_DIR}/setup_vcan.sh" \
    "${STAGING_DIR}/scripts/setup_vcan.sh"
install -m 0755 "${SCRIPT_DIR}/prepare_environment.sh" \
    "${STAGING_DIR}/scripts/prepare_environment.sh"
install -m 0644 "${PROJECT_ROOT}/LICENSE" "${STAGING_DIR}/LICENSE"
install -m 0644 "${PROJECT_ROOT}/THIRD_PARTY_NOTICES.md" \
    "${STAGING_DIR}/THIRD_PARTY_NOTICES.md"

install -m 0755 "${MUJOCO_RUNTIME}" "${STAGING_DIR}/lib/${MUJOCO_SONAME}"
if [[ "${BUILD_VIEWER}" == ON ]]; then
    install -m 0755 "${GLFW_RUNTIME}" "${STAGING_DIR}/lib/${GLFW_SONAME}"
fi

if [[ -f "${MUJOCO_ROOT}/LICENSE" ]]; then
    install -m 0644 "${MUJOCO_ROOT}/LICENSE" \
        "${STAGING_DIR}/licenses/mujoco/LICENSE"
fi
if [[ -f "${MUJOCO_ROOT}/THIRD_PARTY_NOTICES" ]]; then
    install -m 0644 "${MUJOCO_ROOT}/THIRD_PARTY_NOTICES" \
        "${STAGING_DIR}/licenses/mujoco/THIRD_PARTY_NOTICES"
fi
if [[ "${BUILD_VIEWER}" == ON && -f /usr/share/doc/libglfw3/copyright ]]; then
    install -m 0644 /usr/share/doc/libglfw3/copyright \
        "${STAGING_DIR}/licenses/glfw/copyright"
fi

MODEL_DIRECTORY_NAME="$(basename -- "${MODEL_DIR}")"
PACKAGE_MODEL_PATH="mjcf/${MODEL_DIRECTORY_NAME}/$(basename -- "${MODEL_PATH}")"
install -d "${STAGING_DIR}/mjcf"
cp -a -- "${MODEL_DIR}" "${STAGING_DIR}/mjcf/${MODEL_DIRECTORY_NAME}"

copy_config_resources() {
    local key value source relative destination
    while IFS=$'\t' read -r key value; do
        [[ -n "${value}" ]] || continue
        source="$(absolute_path "${value}" "${CONFIG_DIR}")"
        source="$(realpath -e -- "${source}")"
        relative="$(realpath --relative-to="${PROJECT_ROOT}" -- "${source}")"
        if [[ "${relative}" == ../* || "${relative}" == ".." ]]; then
            fail "${key} must be inside the project for runtime packaging: ${source}"
        fi
        destination="${STAGING_DIR}/${relative}"
        install -d "$(dirname -- "${destination}")"
        install -m 0644 "${source}" "${destination}"
    done < <(
        awk -F= '
            /^[[:space:]]*(sensor_model|calibration_file)[[:space:]]*=/ {
                key=$1
                gsub(/^[[:space:]]+|[[:space:]]+$/, "", key)
                value=substr($0, index($0, "=") + 1)
                sub(/[[:space:]]*#.*/, "", value)
                gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
                gsub(/^[\047\"]|[\047\"]$/, "", value)
                print key "\t" value
            }
        ' "${CONFIG_PATH}"
    )
}
copy_config_resources

awk -v package_model="${PACKAGE_MODEL_PATH}" -v build_viewer="${BUILD_VIEWER}" '
    /^[[:space:]]*\[/ {
        section=$0
        gsub(/^[[:space:]]*\[|\][[:space:]]*$/, "", section)
    }
    section == "simulation" && /^[[:space:]]*model[[:space:]]*=/ {
        print "model=" package_model
        next
    }
    section == "viewer" && /^[[:space:]]*enabled[[:space:]]*=/ && build_viewer == "OFF" {
        print "enabled=false"
        next
    }
    /^[[:space:]]*mujoco_root[[:space:]]*=/ {
        print "# MuJoCo runtime is bundled in lib/."
        next
    }
    { print }
' "${CONFIG_PATH}" > "${STAGING_DIR}/sharpa_mujoco.conf"

STRIP_TOOL=""
if command -v strip >/dev/null 2>&1; then
    STRIP_TOOL="$(command -v strip)"
fi
if [[ -n "${STRIP_TOOL}" ]]; then
    "${STRIP_TOOL}" --strip-unneeded "${STAGING_DIR}/bin/sharpa_mujoco_simulator"
    "${STRIP_TOOL}" --strip-unneeded "${STAGING_DIR}/bin/sharpa_mujoco_environment"
    "${STRIP_TOOL}" --strip-unneeded "${STAGING_DIR}/lib/${MUJOCO_SONAME}"
fi

validate_elf_arch readelf "${STAGING_DIR}/bin/sharpa_mujoco_simulator" "${PACKAGE_ARCH}"
validate_elf_arch readelf "${STAGING_DIR}/bin/sharpa_mujoco_environment" "${PACKAGE_ARCH}"
validate_elf_arch readelf "${STAGING_DIR}/lib/${MUJOCO_SONAME}" "${PACKAGE_ARCH}"

missing_libraries="$(
    LD_LIBRARY_PATH="${STAGING_DIR}/lib" \
        ldd "${STAGING_DIR}/bin/sharpa_mujoco_simulator" |
        awk '/not found/ { print $1 }' |
        sort -u
)"
[[ -z "${missing_libraries}" ]] ||
    fail "packaged simulator still has missing libraries: ${missing_libraries}"

mv -- "${STAGING_DIR}" "${PACKAGE_DIR}"
trap - EXIT

tar -C "${OUTPUT_DIR}" -cf - "${PACKAGE_NAME}" |
    xz -T0 -3 -c > "${ARCHIVE_PATH}"

PACKAGE_SIZE="$(du -sh "${PACKAGE_DIR}" | awk '{ print $1 }')"
ARCHIVE_SIZE="$(du -h "${ARCHIVE_PATH}" | awk '{ print $1 }')"

echo "Removing intermediate build directory: ${BUILD_DIR}"
rm -rf -- "${BUILD_DIR}"

echo
echo "Runtime package created successfully."
echo "  directory: ${PACKAGE_DIR} (${PACKAGE_SIZE})"
echo "  archive:   ${ARCHIVE_PATH} (${ARCHIVE_SIZE})"
echo "  run:       ${PACKAGE_DIR}/run.sh"
