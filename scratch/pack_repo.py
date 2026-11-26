#!/usr/bin/env python3
"""Pack the Folkrace repository into size-capped text bundles.

Usage:
  python scratch/pack_repo.py                 # Core firmware, planner, brain & tools
  python scratch/pack_repo.py --include-guis  # Also include desktop/web bench GUIs
"""

import argparse
import pathlib
import shutil

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = ROOT / "scratch" / "bundles"
MAX_BYTES = 360_000

EXTS = {".py", ".cpp", ".c", ".h", ".hpp", ".md", ".json", ".ini", ".lua", ".sh", ".proto"}

ALWAYS_SKIP_DIRS = {
    ".git",
    ".pio",
    ".vscode",
    "__pycache__",
    "scratch",
    "data",
    "meshes",
    "osqp_codegen_sl_mpc",
    "VL53L8CX",
}

GUI_SKIP_DIRS = {
    "simplefoc_mt6816_suite",
    "imu_gui",
    "servo_test",
    "test_pwm",
    "track_builder",
    "gui",
}

ALWAYS_SKIP_FILES = {
    "web_debug_gui.html",
    "folkrace_bundle.txt",
    "folkrace_core.txt",
}

ORDERED_TARGETS = [
    "README.md",
    "platformio.ini",
    ".agents",
    "brain",
    "2_pi4",
    "3_esp32",
    "4_stm32",
    "5_webots",
    "tools",
    "1_launchers",
]


def collect_files(include_guis: bool) -> list[pathlib.Path]:
    skip_dirs = set(ALWAYS_SKIP_DIRS)
    if not include_guis:
        skip_dirs |= GUI_SKIP_DIRS

    files: list[pathlib.Path] = []
    for target in ORDERED_TARGETS:
        path = ROOT / target
        if not path.exists():
            continue
        candidates = [path] if path.is_file() else sorted(path.rglob("*"))
        for file in candidates:
            if not file.is_file():
                continue
            rel_parts = file.relative_to(ROOT).parts
            if any(part in skip_dirs for part in rel_parts):
                continue
            if file.name in ALWAYS_SKIP_FILES:
                continue
            if not include_guis and (file.name.endswith("_gui.py") or file.name.startswith("gui_")):
                continue
            if file.suffix.lower() in EXTS:
                files.append(file)
    return files


def bundle_name(chunk: list[tuple[str, str]], idx: str, total: str) -> str:
    first_top = chunk[0][0].split("/")[0]
    last_top = chunk[-1][0].split("/")[0]
    span = first_top if first_top == last_top else f"{first_top}_to_{last_top}"
    return f"folkrace_{idx}of{total}_{span}.txt"


def render_bundle(name: str, chunk: list[tuple[str, str]]) -> str:
    toc = "\n".join(f"  - {rel}" for rel, _ in chunk)
    header = f"=== BUNDLE: {name} ({len(chunk)} files) ===\n{toc}\n\n"
    return header + "".join(block for _, block in chunk)


def main() -> None:
    parser = argparse.ArgumentParser(description="Bundle Folkrace repo into size-capped text files")
    parser.add_argument("--include-guis", action="store_true", help="Include GUI and bench scripts")
    parser.add_argument(
        "--max-kb",
        type=int,
        help=f"Override the default bundle ceiling of {MAX_BYTES:,} bytes",
    )
    args = parser.parse_args()

    max_bytes = args.max_kb * 1024 if args.max_kb is not None else MAX_BYTES
    if max_bytes <= 0:
        parser.error("--max-kb must be a positive integer")

    if OUT_DIR.exists():
        shutil.rmtree(OUT_DIR)
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    files = collect_files(include_guis=args.include_guis)
    chunks: list[list[tuple[str, str]]] = []
    current: list[tuple[str, str]] = []
    placeholder_digits = len(str(max(1, len(files))))

    for file in files:
        rel = file.relative_to(ROOT).as_posix()
        body = file.read_text(encoding="utf-8", errors="replace")
        block = f"=== FILE: {rel} ===\n{body}\n\n"
        candidate = current + [(rel, block)]
        placeholder = bundle_name(candidate, "9" * placeholder_digits, "9" * placeholder_digits)
        candidate_size = len(render_bundle(placeholder, candidate).encode("utf-8"))

        if candidate_size > max_bytes:
            if current:
                chunks.append(current)
            current = [(rel, block)]
            placeholder = bundle_name(current, "9" * placeholder_digits, "9" * placeholder_digits)
            if len(render_bundle(placeholder, current).encode("utf-8")) > max_bytes:
                raise ValueError(f"{rel} is too large to fit in a {max_bytes}-byte bundle")
        else:
            current = candidate

    if current:
        chunks.append(current)

    total = len(chunks)
    print(f"Generated {total} bundle file(s) in {OUT_DIR.relative_to(ROOT)}/:")
    for idx, chunk in enumerate(chunks, 1):
        name = bundle_name(chunk, str(idx), str(total))
        content = render_bundle(name, chunk)
        size = len(content.encode("utf-8"))
        if size > max_bytes:
            raise ValueError(f"{name} exceeds the {max_bytes}-byte bundle ceiling")
        out_path = OUT_DIR / name
        out_path.write_text(content, encoding="utf-8")
        print(f"  {name:<48} {size / 1024:6.1f} KB ({len(chunk)} files)")


if __name__ == "__main__":
    main()
