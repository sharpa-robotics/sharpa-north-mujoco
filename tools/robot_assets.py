#!/usr/bin/env python3
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
"""Check/copy the official scene's dependency closure without flattening paths."""
import argparse
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / "mjcf/sharpa-assets"
SCENE = Path("d01/d01-w/ha1.2/mjcf/NTH-HA1-scene-precise-collision.xml")


def dependencies(root, scene):
    root = root.resolve()
    files = set()

    def visit(path, xml=False):
        path = path.resolve()
        if not path.is_relative_to(root):
            raise ValueError(f"Asset reference escapes repository: {path}")
        if path in files:
            return
        if not path.is_file():
            raise ValueError(f"Missing asset: {path}")
        with path.open("rb") as stream:
            if stream.read(80).startswith(b"version https://git-lfs.github.com/spec/v1"):
                raise ValueError(f"Git LFS content not downloaded: {path}")
        files.add(path)
        if xml:
            document = ET.parse(path).getroot()
            compiler = document.find("compiler")
            dirs = compiler.attrib if compiler is not None else {}
            for item in document.iter():
                filename = item.get("file")
                if not filename:
                    continue
                if item.tag == "include":
                    visit(path.parent / filename, True)
                elif item.tag in ("mesh", "texture", "hfield", "skin"):
                    directory = dirs.get("texturedir" if item.tag == "texture" else "meshdir",
                                         dirs.get("assetdir", ""))
                    visit(path.parent / directory / filename)

    visit(root / scene, True)
    for notice in ("LICENSE.txt", "NOTICE.txt"):
        visit(root / notice)
    return sorted(files)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("check", "copy", "list"))
    parser.add_argument("destination", nargs="?", type=Path)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--scene", type=Path, default=SCENE)
    args = parser.parse_args()
    try:
        files = dependencies(args.root, args.scene)
        for source in files:
            relative = source.relative_to(args.root.resolve())
            if args.action == "copy":
                if args.destination is None:
                    raise ValueError("copy requires a destination directory")
                destination = args.destination / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
            elif args.action == "list":
                print(relative)
        if args.action != "list":
            print(f"D01-W precise assets ready: {len(files)} files")
    except (OSError, ValueError, ET.ParseError) as error:
        print(f"{error}\nRun ./scripts/update_submodules.sh from the project root.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
