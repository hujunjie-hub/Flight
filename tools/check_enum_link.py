"""Check FLIGHT-14 diagnostics and runtime inputs for the CubeCLT Release link."""
import argparse
from collections import Counter
from hashlib import file_digest
import json
from pathlib import Path, PurePosixPath
import re


BASELINE = Path(__file__).resolve().parents[1] / "tests/abi/release_enum_link_baseline.json"
ENUM_WARNING = re.compile(r"\bwarning:.*\benums?\b", re.IGNORECASE)
KNOWN_WARNING = re.compile(
    r".*[/\\]ld(?:\.exe)?: warning: (.+) uses 32-bit enums yet the output is to "
    r"use variable-size enums; use of enum values across objects may fail"
)
ABSOLUTE_PATH = re.compile(r"^(?:[A-Za-z]:/|/)")
ARCHIVE_HEADER = "Archive member included to satisfy reference by file (symbol)"


def runtime_inputs(map_text):
    body, separator, cross_refs = map_text.replace("\\", "/").partition("\nCross Reference Table\n")
    if not separator or not re.search(r"^Symbol\s+File\s*$", cross_refs, re.MULTILINE):
        raise ValueError("linker map is missing its cross-reference table")
    if not body.startswith(ARCHIVE_HEADER + "\n"):
        raise ValueError("linker map is missing its archive member table")

    loads = [line.removeprefix("LOAD ") for line in body.splitlines() if line.startswith("LOAD ")]
    runtime_paths = [path for path in loads if ABSOLUTE_PATH.match(path) or path.endswith(".a")]
    names = {path: PurePosixPath(path).name for path in runtime_paths}
    hashes = {}
    for path in names:
        with Path(path).open("rb") as item:
            hashes[path] = file_digest(item, "sha256").hexdigest()

    def runtime_name(path):
        if path in names:
            return names[path]
        archive, _, member = path.rpartition("(")
        if archive in names and member.endswith(")"):
            return f"{names[archive]}({member}"
        return None

    member_section = body.split("\nDiscarded input sections", 1)[0]
    members = sorted(name for line in member_section.splitlines() if line.endswith(")")
                     and not line[0].isspace() and (name := runtime_name(line)))

    symbols = {}
    for line in cross_refs.splitlines():
        if not line.strip() or line.split() == ["Symbol", "File"]:
            continue
        if line[0].isspace():
            if not symbols:
                raise ValueError("linker map has a cross-reference without a symbol")
            symbols[symbol].append(runtime_name(line.strip()))
        else:
            symbol, path = line.split(maxsplit=1)
            symbols[symbol] = [runtime_name(path)]

    # GNU ld lists the definition first, when present. A dash marks a file
    # outside the runtime. Keep all participating runtime objects and direction.
    boundary = []
    for symbol, files in symbols.items():
        if None in files and any(files):
            definition = files[0] or "-"
            runtime_references = sorted({name for name in files[1:] if name})
            boundary.append(f"{symbol}: {' '.join([definition, *runtime_references])}")

    return {
        "runtime_loads": [{"file": names[path], "sha256": hashes[path]} for path in runtime_paths],
        "runtime_members": members,
        "runtime_boundary": sorted(boundary),
    }


def check_enum_link(log, map_text, baseline):
    objects = []
    problems = []
    warning_count = 0
    fresh_link = "LINK Flight.elf" in log.splitlines()
    if not fresh_link:
        problems.append("no fresh LINK Flight.elf in the SCons log")

    for line in log.splitlines():
        if ENUM_WARNING.search(line):
            warning_count += 1
            match = KNOWN_WARNING.fullmatch(line)
            if match:
                objects.append(match[1].replace("\\", "/"))
            else:
                problems.append(f"unrecognized enum diagnostic: {line}")

    expected = Counter(baseline["enum_warning_objects"])
    actual = Counter(objects)
    if actual != expected:
        problems.append("enum warning objects changed; "
                        f"added={list((actual - expected).elements())[:5]}, "
                        f"missing={list((expected - actual).elements())[:5]}")

    runtime = {}
    try:
        runtime = runtime_inputs(map_text)
        for field, values in runtime.items():
            if values != baseline[field]:
                problems.append(f"{field} differs from the reviewed baseline")
    except (OSError, ValueError) as exc:
        problems.append(f"could not verify linker map: {exc}")

    return {
        "status": "FAIL" if problems else "PASS",
        "enum_warning_count": warning_count,
        "expected_warning_count": sum(expected.values()),
        "enum_warning_objects": sorted(objects),
        "problems": problems,
        **runtime,
    }


def check_files(log_file, map_file):
    return check_enum_link(log_file.read_text(encoding="utf-8-sig", errors="replace"),
                           map_file.read_text(encoding="utf-8", errors="replace"),
                           json.loads(BASELINE.read_text(encoding="utf-8")))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("map", type=Path)
    args = parser.parse_args()
    result = check_files(args.log, args.map)
    print(json.dumps(result, indent=2))
    return int(result["status"] != "PASS")


if __name__ == "__main__":
    raise SystemExit(main())
