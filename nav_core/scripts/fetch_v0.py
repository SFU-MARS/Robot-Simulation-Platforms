"""
Download the offline V0 files listed in robots/v0_release.yaml (assets of a GitHub release of this repository)
into their folder, and check their sha256.  Files already there with the right checksum are kept.

    cd nav_core
    python scripts/fetch_v0.py                 # the V0 of every ready robot profile (base_value)
    python scripts/fetch_v0.py --all           # every listed V0
    python scripts/fetch_v0.py --check         # only check the files that are there

A private fork needs a token: GITHUB_TOKEN=... python scripts/fetch_v0.py
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import urllib.request
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "robots" / "v0_release.yaml"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def profile_files(manifest: dict) -> list:
    """File names of the base_value of every ready profile that the manifest lists."""
    names = []
    for yml in sorted((ROOT / "robots").glob("*.yaml")):
        d = yaml.safe_load(yml.read_text())
        if isinstance(d, dict) and d.get("status") == "ready" and d.get("base_value"):
            name = Path(d["base_value"]).name
            if name in manifest["files"] and name not in names:
                names.append(name)
    return names


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--all", action="store_true", help="every V0 in the manifest, not only the profiles' ones")
    ap.add_argument("--check", action="store_true", help="do not download, only check the files that are there")
    args = ap.parse_args()

    manifest = yaml.safe_load(MANIFEST.read_text())
    folder = ROOT / manifest["folder"]
    folder.mkdir(parents=True, exist_ok=True)
    names = list(manifest["files"]) if args.all else profile_files(manifest)
    base = f"https://github.com/{manifest['repository']}/releases/download/{manifest['tag']}/"
    token = os.environ.get("GITHUB_TOKEN")
    failed = False
    for name in names:
        path, expected = folder / name, manifest["files"][name]
        if path.is_file() and sha256(path) == expected:
            print(f"ok        {path}")
            continue
        if args.check:
            print(f"{'MISMATCH' if path.is_file() else 'MISSING':9s} {path}")
            failed = True
            continue
        request = urllib.request.Request(base + name, headers={"Authorization": f"token {token}"} if token else {})
        tmp = path.with_suffix(path.suffix + ".part")
        try:
            with urllib.request.urlopen(request) as response, open(tmp, "wb") as f:
                while True:
                    chunk = response.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
        except Exception as e:                       # noqa: BLE001 -- report every failure, keep going
            print(f"FAILED    {base + name}: {e}")
            tmp.unlink(missing_ok=True)
            failed = True
            continue
        if sha256(tmp) != expected:
            print(f"MISMATCH  {base + name}: the download does not match robots/v0_release.yaml")
            tmp.unlink()
            failed = True
            continue
        tmp.replace(path)
        print(f"fetched   {path}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
