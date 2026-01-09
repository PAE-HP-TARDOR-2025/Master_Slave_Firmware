#!/usr/bin/env python3
"""Build slave firmware variants with different greetings and versions."""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

SLAVE_DIR = Path(__file__).resolve().parent / "slave"

def ensure_idf():
    """Check if idf.py is available."""
    idf_py = shutil.which("idf.py")
    if idf_py is None:
        sys.exit("ERROR: idf.py not found in PATH. Run ESP-IDF export script first.")
    return Path(idf_py)

def parse_greeting(value):
    """Parse NAME:TEXT:VERSION format."""
    parts = value.split(":")
    if len(parts) < 2:
        raise argparse.ArgumentTypeError("Format: NAME:TEXT:VERSION")
    name = parts[0].strip()
    text = parts[1].strip()
    version = int(parts[2].strip()) if len(parts) >= 3 else 1
    if not name or not text:
        raise argparse.ArgumentTypeError("Name and text cannot be empty")
    return name, text, version

def build_firmware(name, greeting, version, idf_py):
    """Build firmware with specific greeting and version."""
    print(f"\n{'='*60}")
    print(f"Building: {name} (version {version})")
    print(f"Greeting: {greeting}")
    print(f"{'='*60}\n")
    
    build_dir = SLAVE_DIR / f"build-{name}"
    output_dir = Path(__file__).resolve().parent / "binaries"
    output_dir.mkdir(exist_ok=True)
    
    # Clean previous build
    if build_dir.exists():
        print(f"Cleaning {build_dir}...")
        shutil.rmtree(build_dir)
    
    # Create temporary sdkconfig with version
    sdkconfig_version = SLAVE_DIR / "sdkconfig.version"
    with open(sdkconfig_version, "w") as f:
        f.write(f"CONFIG_FIRMWARE_VERSION={version}\n")
        f.write(f'CONFIG_FIRMWARE_GREETING="{greeting}"\n')
    
    # Build with version defines
    cmd = [
        str(idf_py), 
        "-B", str(build_dir),
        "-D", f"SDKCONFIG_DEFAULTS={sdkconfig_version}",
        "build"
    ]
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=SLAVE_DIR)
    if result.returncode != 0:
        sys.exit(f"Build failed for {name}")
    
    # Generate app-only binary (for OTA)
    elf_file = build_dir / "slave.elf"
    app_bin = build_dir / "slave_app.bin"
    
    esptool_cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32",
        "elf2image",
        "--flash_mode", "dio",
        "--flash_freq", "40m",
        "--flash_size", "4MB",
        "-o", str(app_bin),
        str(elf_file)
    ]
    print(f"\nGenerating OTA binary: {app_bin.name}")
    result = subprocess.run(esptool_cmd, cwd=SLAVE_DIR)
    if result.returncode != 0:
        sys.exit(f"Failed to generate app binary for {name}")
    
    # Copy to output directory
    output_file = output_dir / f"{name}.bin"
    shutil.copy2(app_bin, output_file)
    print(f"✓ Created: {output_file}")
    
    # Cleanup temporary sdkconfig
    if sdkconfig_version.exists():
        sdkconfig_version.unlink()
    
    return output_file

def main():
    parser = argparse.ArgumentParser(
        description="Build slave firmware variants",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Build solo una versión (recomendado)
  python build_slave_bins.py --greeting "version1:Firmware v1:1"
  
  # Build múltiples versiones
  python build_slave_bins.py --greeting "version1:Firmware v1:1" --greeting "version2:Firmware v2:2"
  
  # Sin argumentos: build solo version1 (por defecto)
  python build_slave_bins.py
        """
    )
    parser.add_argument(
        "--greeting",
        type=parse_greeting,
        action="append",
        help="NAME:TEXT:VERSION (can be specified multiple times)"
    )
    
    args = parser.parse_args()
    
    if not args.greeting:
        # Default: build solo version1 (firmware actual)
        args.greeting = [
            ("version1", "Firmware version 1", 1),
        ]
    
    idf_py = ensure_idf()
    
    print(f"Slave directory: {SLAVE_DIR}")
    print(f"Building {len(args.greeting)} firmware variant(s)...\n")
    
    for name, text, version in args.greeting:
        build_firmware(name, text, version, idf_py)
    
    print(f"\n{'='*60}")
    print("All builds completed successfully!")
    print(f"Binaries in: {Path(__file__).resolve().parent / 'binaries'}")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
