#!/usr/bin/env python3
"""Utility that builds greeting-specific ESP-IDF binaries for the dummy slave."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
PROJECT_DIR = REPO_ROOT / "demoslave"
DEFAULT_GREETING_PAIRS = (
    ("hello", "Hello from slave"),
    ("bye", "Bye from slave"),
)


def parse_greeting(value: str) -> tuple[str, str]:
    if ":" not in value:
        raise argparse.ArgumentTypeError("Greeting must be NAME:TEXT")
    name, text = value.split(":", 1)
    name = name.strip()
    text = text.strip()
    if not name:
        raise argparse.ArgumentTypeError("Greeting name cannot be empty")
    if not text:
        raise argparse.ArgumentTypeError("Greeting text cannot be empty")
    return name, text


def parse_greeting_with_version(value: str) -> tuple[str, str, int]:
    """Parse NAME:TEXT or NAME:TEXT:VERSION format."""
    parts = value.split(":")
    if len(parts) < 2:
        raise argparse.ArgumentTypeError("Greeting must be NAME:TEXT or NAME:TEXT:VERSION")
    name = parts[0].strip()
    text = parts[1].strip()
    version = 1
    if len(parts) >= 3:
        try:
            version = int(parts[2].strip())
        except ValueError:
            raise argparse.ArgumentTypeError(f"Version must be an integer, got '{parts[2]}'")
    if not name:
        raise argparse.ArgumentTypeError("Greeting name cannot be empty")
    if not text:
        raise argparse.ArgumentTypeError("Greeting text cannot be empty")
    return name, text, version


def ensure_idf() -> tuple[Path, str]:
    idf_py = shutil.which("idf.py")
    if idf_py is None:
        sys.exit("idf.py not found in PATH. Run the ESP-IDF export script first.")

    python_cmd = os.environ.get("PYTHON") or sys.executable
    return Path(idf_py), python_cmd


def run_build(
    build_name: str,
    greeting: str,
    version: int,
    target: str,
    output_dir: Path,
    idf_py: Path,
    python_cmd: str,
) -> Path:
    build_dir = PROJECT_DIR / f"build-{build_name}"
    ensure_fresh_build_dir(build_dir)
    env = os.environ.copy()
    env["SLAVE_GREETING_OVERRIDE"] = greeting
    env["IDF_TARGET"] = target

    # Create temporary sdkconfig.defaults to set the version
    sdkconfig_defaults = PROJECT_DIR / "sdkconfig.defaults"
    original_defaults = None
    if sdkconfig_defaults.exists():
        original_defaults = sdkconfig_defaults.read_text()
    
    try:
        # Append version setting to sdkconfig.defaults
        new_defaults = (original_defaults or "") + f"\nCONFIG_DEMO_SLAVE_FW_VERSION={version}\n"
        sdkconfig_defaults.write_text(new_defaults)
        
        cmd = [
            python_cmd,
            str(idf_py),
            "-C",
            str(PROJECT_DIR),
            "-B",
            str(build_dir),
            "build",
        ]
        print(f"[BUILD] greeting='{greeting}' version={version} -> {build_dir}")
        subprocess.run(cmd, check=True, env=env)
    finally:
        # Restore original sdkconfig.defaults
        if original_defaults is not None:
            sdkconfig_defaults.write_text(original_defaults)
        elif sdkconfig_defaults.exists():
            sdkconfig_defaults.unlink()

    image = locate_project_image(build_dir)
    if image is None:
        sys.exit(
            "project_name.txt or project_description.json did not yield a firmware image. "
            f"Run 'idf.py reconfigure' inside {build_dir} and re-run this script."
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    dest = output_dir / f"{build_name}.bin"
    shutil.copy2(image, dest)
    print(f"[BUILD] Copied {image} -> {dest}")
    return dest


def ensure_fresh_build_dir(build_dir: Path) -> None:
    """Remove an existing build dir if it belongs to another checkout."""

    cache_file = build_dir / "CMakeCache.txt"
    if not cache_file.exists():
        return

    try:
        cache_contents = cache_file.read_text(errors="ignore")
    except OSError:
        return

    current_project = str(PROJECT_DIR.resolve())
    if current_project in cache_contents:
        return

    print(
        f"[BUILD] Removing stale build directory {build_dir} (cache for a different project location)"
    )
    shutil.rmtree(build_dir, ignore_errors=True)


def locate_project_image(build_dir: Path) -> Path | None:
    """Find the built .bin using either project_name.txt or project_description.json."""

    project_name_file = build_dir / "project_name.txt"
    if project_name_file.exists():
        project_name = project_name_file.read_text().strip()
        if project_name:
            candidate = build_dir / f"{project_name}.bin"
            if candidate.exists():
                return candidate

    desc_file = build_dir / "project_description.json"
    if desc_file.exists():
        try:
            data = json.loads(desc_file.read_text())
        except (OSError, json.JSONDecodeError):
            pass
        else:
            app_bin = data.get("app_bin")
            if app_bin:
                candidate = build_dir / app_bin
                if candidate.exists():
                    return candidate
            project_name = data.get("project_name")
            if project_name:
                candidate = build_dir / f"{project_name}.bin"
                if candidate.exists():
                    return candidate

    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--idf-target",
        default="esp32",
        help="IDF target passed via IDF_TARGET (default: %(default)s)",
    )
    parser.add_argument(
        "--greeting",
        action="append",
        type=parse_greeting_with_version,
        help="Greeting build spec in the form NAME:TEXT or NAME:TEXT:VERSION (can be repeated)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "artifacts",
        help="Directory where the resulting .bin files are copied",
    )

    args = parser.parse_args()
    idf_py, python_cmd = ensure_idf()

    # Default greetings with version 1
    greetings = args.greeting if args.greeting else [("hello", "Hello from slave", 1), ("bye", "Bye from slave", 1)]
    if not greetings:
        sys.exit("No greetings provided")

    results = []
    for name, text, version in greetings:
        dest = run_build(name, text, version, args.idf_target, args.output_dir, idf_py, python_cmd)
        results.append(dest)

    print("[BUILD] Completed:")
    for dest in results:
        print(f" - {dest}")


if __name__ == "__main__":
    main()
