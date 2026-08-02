#!/usr/bin/env python3
"""Apply only track-path-related model blocks from a commit/world SDF into the current world.

This is intended for cases where you want the track tiles from a specific commit's
navi_factory SDF, but not the unrelated warehouse props or people models.

Examples:
    python3 tools/apply_commit_track_models.py \
        --commit 46a6e341e781ec6b028138b0a32a94f90bc25100 \
        --source-world worlds/navi_factory/world/navi_factory/navi_factory.sdf \
        --world worlds/navi_factory/world/navi_factory/navi_factory.sdf

    python3 tools/apply_commit_track_models.py \
        --source-file /tmp/navi_factory.sdf \
        --world worlds/navi_factory/world/navi_factory/navi_factory.sdf
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict

MODEL_BLOCK_RE = re.compile(r"<model name='([^']+)'>.*?</model>", re.DOTALL)
TARGET_NAME_RE = re.compile(r"^(track_|hand_signal_spot)$")


def _read_source_sdf(source_file: Path | None, commit: str | None, source_world: str | None) -> str:
    if source_file is not None:
        return source_file.read_text(encoding="utf-8")

    if commit is not None:
        if source_world is None:
            raise ValueError("--source-world is required when --commit is used")
        result = subprocess.run(
            ["git", "show", f"{commit}:{source_world}"],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or "failed to read commit file")
        return result.stdout

    raise ValueError("either --source-file or --commit/--source-world must be provided")


def extract_target_models(sdf_text: str) -> Dict[str, str]:
    blocks: Dict[str, str] = {}
    for match in MODEL_BLOCK_RE.finditer(sdf_text):
        name = match.group(1)
        if TARGET_NAME_RE.match(name):
            blocks[name] = match.group(0)
    return blocks


def replace_target_models(world_text: str, model_blocks: Dict[str, str]) -> str:
    if not model_blocks:
        raise ValueError("no target models found")

    for name in sorted(model_blocks):
        pattern = re.compile(rf"<model name='{re.escape(name)}'>.*?</model>", re.DOTALL)
        world_text = pattern.sub("", world_text)

    insertion = []
    for name in sorted(model_blocks):
        block_text = model_blocks[name]
        insertion.append(block_text)

    block_payload = "\n\n".join(insertion)
    indented_payload = "\n".join("    " + line if line else "" for line in block_payload.splitlines())
    if indented_payload:
        indented_payload += "\n"

    if "</world>" not in world_text:
        raise ValueError("world file does not contain </world>")

    return world_text.replace("</world>", f"{indented_payload}</world>", 1)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--commit", default=None, help="git commit SHA to read the source SDF from")
    parser.add_argument("--source-world", default=None, help="path inside the repo for the source SDF")
    parser.add_argument("--source-file", default=None, help="local source SDF file path")
    parser.add_argument("--world", required=True, help="target world SDF file to update")
    args = parser.parse_args()

    source_sdf = _read_source_sdf(Path(args.source_file) if args.source_file else None, args.commit, args.source_world)
    model_blocks = extract_target_models(source_sdf)
    target_world = Path(args.world)
    world_text = target_world.read_text(encoding="utf-8")
    updated_text = replace_target_models(world_text, model_blocks)
    target_world.write_text(updated_text, encoding="utf-8")
    print(f"updated {target_world} with {len(model_blocks)} track-related model blocks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
