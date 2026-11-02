#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"

BUILD_DIR="$PROJECT_ROOT/build-linux"
BUILD_DIR_EXPLICIT=false
BUILD_TYPE="RelWithDebInfo"
BUILD_TESTS=OFF
BUILD_VIEWER=ON
RUN_TESTS=false
CLEAN=false
JOBS="${JOBS:-$(nproc)}"
STAGE_PLATFORM=ON
PLATFORM_MODE=with

usage() {
    cat <<'EOF'
Usage: ./scripts/build.sh [options]

Configure and build Sharpa MuJoCo incrementally by default.
Existing build outputs and the CMake cache are reused.
Tests are built and run only when --tests is specified.

Options:
  -c, --clean           Remove a verified build directory before rebuilding
  -d, --debug           Build Debug instead of RelWithDebInfo
      --release         Build Release
      --tests           Build and run tests
      --no-tests        Do not build or run tests (default)
      --no-viewer       Build without the Linux Viewer/GLFW dependency
      --with-platform   Require and stage the Platform binary bundle (default)
      --without-platform Build without Platform in ./build-linux-no-platform
  -j, --jobs N          Parallel build jobs (default: nproc)
  -B, --build-dir PATH  Build directory (default: ./build-linux)
  -h, --help            Show this help

Examples:
  ./scripts/build.sh
  ./scripts/build.sh --tests
  ./scripts/build.sh --without-platform
  ./scripts/build.sh --no-viewer
  ./scripts/build.sh --clean
  ./scripts/build.sh --debug -j 8
EOF
}

validate_clean_directory() {
    local resolved_build_dir cache_file cached_source cached_build
    resolved_build_dir="$(realpath -m -- "$BUILD_DIR")"
    if [[ "$resolved_build_dir" == / || "$PROJECT_ROOT" == "$resolved_build_dir" ||
          "$PROJECT_ROOT" == "$resolved_build_dir/"* ||
          -e "$resolved_build_dir/CMakeLists.txt" ]]; then
        echo "error: refusing to clean a source directory or an ancestor of the project: $BUILD_DIR" >&2
        exit 2
    fi
    if [[ "$(realpath -ms -- "$BUILD_DIR")" != "$resolved_build_dir" ]]; then
        echo "error: refusing to clean through a symbolic link: $BUILD_DIR" >&2
        exit 2
    fi
    BUILD_DIR="$resolved_build_dir"

    # A first build has no directory to remove. Existing directories must be
    # identified by CMake as builds of this checkout at this exact location.
    [[ -e "$BUILD_DIR" ]] || return 0
    cache_file="$BUILD_DIR/CMakeCache.txt"
    if [[ ! -d "$BUILD_DIR" || ! -f "$cache_file" || -L "$cache_file" ]]; then
        echo "error: refusing to clean an unverified build directory (a regular CMakeCache.txt is required): $BUILD_DIR" >&2
        exit 2
    fi
    cached_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache_file")"
    cached_build="$(sed -n 's/^CMAKE_CACHEFILE_DIR:INTERNAL=//p' "$cache_file")"
    if [[ "$cached_source" != /* || "$cached_build" != /* ]] ||
       [[ "$(realpath -m -- "$cached_source")" != "$PROJECT_ROOT" ||
          "$(realpath -m -- "$cached_build")" != "$BUILD_DIR" ]]; then
        echo "error: refusing to clean a directory whose CMake cache does not match this project and build path: $BUILD_DIR" >&2
        exit 2
    fi
}

check_platform_bundle() {
    [[ "$STAGE_PLATFORM" == ON ]] || return 0

    local platform_arch
    case "$(uname -m)" in
        x86_64|amd64) platform_arch="x86_64" ;;
        aarch64|arm64) platform_arch="aarch64" ;;
        *) return 0 ;; # CMake will report unsupported architectures.
    esac

    local relative_archive="third_party/sharpa_platform/linux-$platform_arch/sharpa-platform.tar.gz"
    local archive="$PROJECT_ROOT/$relative_archive"
    local first_line=""

    if [[ -f "$archive" ]]; then
        IFS= read -r first_line < "$archive" || true
    fi

    if [[ "$first_line" == "version https://git-lfs.github.com/spec/v1" ]]; then
        cat >&2 <<EOF
error: Sharpa Platform exists, but its runtime archive has not been downloaded.

The file below is only a small Git LFS pointer (a download placeholder), not
the real .tar.gz archive:
  $archive

EOF
        if command -v git-lfs >/dev/null 2>&1; then
            cat >&2 <<EOF
Git LFS is installed. Download the real archive from the repository root:
  git lfs install --local
  git lfs pull --include="$relative_archive" --exclude=""

EOF
        else
            cat >&2 <<EOF
Install Git LFS, then download the real archive from the repository root:
  sudo apt-get install git-lfs
  git lfs install --local
  git lfs pull --include="$relative_archive" --exclude=""

EOF
        fi
        cat >&2 <<'EOF'
If you only need MuJoCo and do not need Sharpa Platform, use:
  ./scripts/build.sh --without-platform
EOF
        exit 1
    fi
}

while (($# > 0)); do
    case "$1" in
        -c|--clean)
            CLEAN=true
            shift
            ;;
        -d|--debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --release)
            BUILD_TYPE="Release"
            shift
            ;;
        --tests)
            BUILD_TESTS=ON
            RUN_TESTS=true
            shift
            ;;
        --no-tests)
            BUILD_TESTS=OFF
            RUN_TESTS=false
            shift
            ;;
        --no-viewer)
            BUILD_VIEWER=OFF
            shift
            ;;
        --with-platform)
            STAGE_PLATFORM=ON
            PLATFORM_MODE=with
            shift
            ;;
        --without-platform)
            STAGE_PLATFORM=OFF
            PLATFORM_MODE=without
            shift
            ;;
        -j|--jobs)
            if (($# < 2)); then
                echo "error: $1 requires a value" >&2
                exit 2
            fi
            JOBS="$2"
            shift 2
            ;;
        -B|--build-dir)
            if (($# < 2)); then
                echo "error: $1 requires a value" >&2
                exit 2
            fi
            if [[ -z "$2" ]]; then
                echo "error: $1 requires a non-empty path" >&2
                exit 2
            fi
            BUILD_DIR="$2"
            [[ "$BUILD_DIR" = /* ]] || BUILD_DIR="$PROJECT_ROOT/$BUILD_DIR"
            BUILD_DIR_EXPLICIT=true
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ "$BUILD_DIR_EXPLICIT" == false && "$PLATFORM_MODE" == without ]]; then
    BUILD_DIR="$PROJECT_ROOT/build-linux-no-platform"
fi

if ! [[ "$JOBS" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: --jobs must be a positive integer" >&2
    exit 2
fi

if [[ "$CLEAN" == true ]]; then
    validate_clean_directory
fi

check_platform_bundle

if [[ "$CLEAN" == true && -d "$BUILD_DIR" ]]; then
    echo "Removing $BUILD_DIR"
    rm -rf -- "$BUILD_DIR"
fi

echo "Configuring Sharpa MuJoCo"
echo "  source:  $PROJECT_ROOT"
echo "  build:   $BUILD_DIR"
echo "  type:    $BUILD_TYPE"
echo "  tests:   $BUILD_TESTS"
echo "  viewer:  $BUILD_VIEWER"
echo "  platform: $STAGE_PLATFORM"
echo "  jobs:    $JOBS"

cmake \
    -S "$PROJECT_ROOT" \
    -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DSHARPA_MUJOCO_BUILD_TESTS="$BUILD_TESTS" \
    -DSHARPA_MUJOCO_BUILD_VIEWER="$BUILD_VIEWER" \
    -DSHARPA_MUJOCO_STAGE_PLATFORM="$STAGE_PLATFORM"

cmake --build "$BUILD_DIR" --parallel "$JOBS"

if [[ "$RUN_TESTS" == true ]]; then
    ctest --test-dir "$BUILD_DIR" --output-on-failure
fi

echo
echo "Build completed successfully."
echo "Simulator: $BUILD_DIR/apps/simulator/sharpa_mujoco_simulator"
