#!/usr/bin/env bash
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
asset_path="mjcf/sharpa-assets"
sdk_path="third_party/north_sdk"
selection=both
mode=pinned
mode_set=false
usage() {
    echo "Usage: $0 [--pinned|--remote] [--only assets|sdk]"
    echo "Default: initialize both submodules at the commits recorded by this repository."
}
while (($#)); do
    case "$1" in
        --pinned|--remote)
            if $mode_set && [[ "$mode" != "${1#--}" ]]; then
                echo "--pinned and --remote are mutually exclusive" >&2; exit 2
            fi
            mode="${1#--}"
            mode_set=true
            shift ;;
        --only)
            (($# >= 2)) || { usage >&2; exit 2; }
            case "$2" in assets|sdk) selection="$2" ;; *) usage >&2; exit 2 ;; esac
            shift 2 ;;
        --help|-h) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
done
command -v python3 >/dev/null
paths=()
if [[ "$selection" != sdk ]]; then
    paths+=("$asset_path")
    git lfs version >/dev/null || {
        echo "Git LFS is required. Run ./scripts/install_dependencies.sh first." >&2
        exit 1
    }
fi
[[ "$selection" == assets ]] || paths+=("$sdk_path")
# Check all selected submodules before updating any of them.
for path in "${paths[@]}"; do
    status_paths=(.)
    # Importing the bundled binding creates bytecode in a repository without .gitignore.
    [[ "$path" != "$sdk_path" ]] || status_paths+=(':(exclude)lib/python/north_sdk/__pycache__')
    if [[ -e "$project_root/$path/.git" ]] &&
       [[ -n "$(git -C "$project_root/$path" status --porcelain -- "${status_paths[@]}")" ]]; then
        echo "Submodule has local changes; commit or move them before updating: $path" >&2
        exit 1
    fi
done
remote_flags=()
[[ "$mode" == pinned ]] || remote_flags+=(--remote)
git -C "$project_root" submodule sync -- "${paths[@]}"
GIT_LFS_SKIP_SMUDGE=1 git -C "$project_root" submodule update --init --recursive "${remote_flags[@]}" -- "${paths[@]}"
if [[ "$selection" != sdk ]]; then
    # The submodule does not inherit filters installed locally in the parent repo.
    git -C "$project_root/$asset_path" lfs install --local
    git -C "$project_root/$asset_path" lfs pull \
        --include="d01/common/visual/**,d01/common/precise-collision/**,d01/d01-w/ha1.2/visual/**,d01/d01-w/ha1.2/precise-collision/**" \
        --exclude=""
    python3 "$project_root/tools/robot_assets.py" check
fi
if [[ "$selection" != assets ]]; then
    # The SDK repository is already an unpacked runtime, not an archive or source build.
    python3 - "$project_root/$sdk_path" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
required = ("Version", "BUILD-MANIFEST.json", "SHA256SUMS", "setup.bash", "check.sh",
            "include/north_sdk/NorthSDK.h", "lib/libnorth_sdk.so",
            "lib/cmake/NorthSDK/NorthSDKConfig.cmake", "lib/python/north_sdk/__init__.py")
for name in required:
    path = root / name
    if not path.is_file():
        sys.exit(f"SDK file missing: {path}")
    with path.open("rb") as stream:
        if stream.read(128).startswith(b"version https://git-lfs.github.com/spec/v1"):
            sys.exit(f"SDK file is a Git LFS pointer: {path}")
version = dict(line.split("=", 1) for line in (root / "Version").read_text().splitlines() if "=" in line)
manifest = json.loads((root / "BUILD-MANIFEST.json").read_text())
if not version.get("VERSION") or version["VERSION"] != manifest.get("version"):
    sys.exit("SDK Version and BUILD-MANIFEST.json disagree")
if not any((root / "lib/python/north_sdk").glob("_north_sdk*.so")):
    sys.exit("SDK Python binding missing")
print("SDK version: " + version["VERSION"])
PY
    (cd "$project_root/$sdk_path" && sha256sum --check --quiet SHA256SUMS)
    # Keep runtime bytecode out of parent/submodule status without changing SDK files.
    exclude="$(git -C "$project_root/$sdk_path" rev-parse --absolute-git-dir)/info/exclude"
    mkdir -p "$(dirname -- "$exclude")"
    touch "$exclude"
    if [[ "$(cat "$exclude")" != *'/lib/python/north_sdk/__pycache__/'* ]]; then
        printf '\n/lib/python/north_sdk/__pycache__/\n' >> "$exclude"
    fi
fi
for path in "${paths[@]}"; do
    echo "$path commit: $(git -C "$project_root/$path" rev-parse HEAD)"
done
