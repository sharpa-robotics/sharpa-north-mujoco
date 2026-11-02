#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "error: Sharpa MuJoCo supports Linux only." >&2
    exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
    echo "error: automatic dependency installation currently requires Ubuntu or Debian." >&2
    echo "Install libglfw3-dev, Git, and Git LFS manually, then fetch the Platform bundle." >&2
    exit 1
fi

case "$(uname -m)" in
    x86_64|amd64) PLATFORM_ARCH="x86_64" ;;
    aarch64|arm64) PLATFORM_ARCH="aarch64" ;;
    *)
        echo "error: no Sharpa Platform bundle is available for this architecture." >&2
        exit 1
        ;;
esac

if ((EUID == 0)); then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
else
    echo "error: sudo is required to install system packages." >&2
    exit 1
fi

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y libglfw3-dev git git-lfs

RELATIVE_ARCHIVE="third_party/sharpa_platform/linux-$PLATFORM_ARCH/sharpa-platform.tar.gz"
ARCHIVE="$PROJECT_ROOT/$RELATIVE_ARCHIVE"

platform_archive_ready() {
    [[ -s "$ARCHIVE" ]] || return 1
    local first_line=""
    LC_ALL=C IFS= read -r -n 128 first_line < "$ARCHIVE" || true
    [[ "$first_line" != "version https://git-lfs.github.com/spec/v1" ]]
}

if ! platform_archive_ready; then
    if ! git -C "$PROJECT_ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        echo "error: downloading the Platform bundle requires a Git checkout of this project." >&2
        echo "Clone the repository with Git, then rerun this script." >&2
        exit 1
    fi

    echo "Downloading the Sharpa Platform bundle for Linux $PLATFORM_ARCH"
    git -C "$PROJECT_ROOT" lfs install --local
    if ! git -C "$PROJECT_ROOT" lfs pull --include="$RELATIVE_ARCHIVE" --exclude=""; then
        echo "error: failed to download the Platform bundle. Check repository access and network connectivity, then rerun this script." >&2
        exit 1
    fi

    if ! platform_archive_ready; then
        echo "error: the Platform bundle is still missing or a Git LFS pointer: $ARCHIVE" >&2
        exit 1
    fi
else
    echo "Sharpa Platform bundle is already downloaded: $ARCHIVE"
fi

echo "Dependencies installed successfully."
