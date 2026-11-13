#!/usr/bin/env python3
"""Host/ARM ABI probe for the Flight production contract headers.

Scope (T191-A-ABI): compile the real production headers in
middleware/ for two targets,
extract sizeof/alignof/offsetof for every public type, compute a canonical ABI
hash, compare the ARM layout against the normative spec layout, and compare the
Host layout against ARM (only pointer-bearing aggregates may differ).

This tool never builds firmware, never generates drivers and never edits the
headers.  It writes:
  tests/abi/evidence/abi_arm.json
  tests/abi/evidence/abi_host.json
  tests/abi/evidence/abi_compare.json
  tests/abi/evidence/commands.json
  tests/abi/evidence/abi_report.md
and, with --emit-lock:
  middleware/contracts/product_contract/abi_lock.h

Run from the Flight project root:
  python tests/abi/abi_report.py --emit-lock   # first baseline
  python tests/abi/abi_report.py               # CI verification
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
HERE = Path(__file__).resolve().parent
CONTRACTS = ROOT / 'middleware/contracts'
EVIDENCE = ROOT / 'tests/abi/evidence'
SPEC_LAYOUT = ROOT / "spec" / "validation" / "abi_layout.json"
ICD = ROOT / "spec" / "02_接口契约与公共数据规范.md"
MESSAGE_REGISTRY = ROOT / "spec" / "contracts" / "message_registry.json"
LOCK_FILE = CONTRACTS / "product_contract/abi_lock.h"

BASELINE = "V1.9.1-GLOBAL-AUDIT-R3"
EXPECTED_SCHEMA = (6, 0)
EXPECTED_PORT_REVISION = 0x00010906
EXPECTED_PRODUCT_REVISION = 0x00010906
EXPECTED_PROFILE_ID = 0x00010001

HEADERS = [
    CONTRACTS / "base_types/contract_version.h",
    CONTRACTS / "base_types/common_base.h",
    CONTRACTS / "ports/control_model_types.h",
    CONTRACTS / "messages/common_messages.h",
    CONTRACTS / "ports/port_types.h",
    CONTRACTS / "ports/port_api.h",
    CONTRACTS / "messages/message_ids.h",
    CONTRACTS / "product_contract/product_contract.h",
]

# Types that are private implementation detail and intentionally not part of
# the production public headers.  They remain in the spec layout only because
# the spec audit compiles every normative C block, including 04 control-private
# snapshots.
PRIVATE_TYPES = {"MpcOutputSnapshot"}

DEFAULT_ARM_BIN = Path(r"D:\STM32CubeCLT\GNU-tools-for-STM32\bin")
DEFAULT_HOST_BIN = Path(r"D:\STM32CubeCLT\mingw64\bin")

ARM_FLAGS = [
    "-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard",
    "-Wall", "-Wextra", "-Werror", "-fno-short-enums",
]
HOST_FLAGS = ["-Wall", "-Wextra", "-Werror"]

commands = []


class AbiError(RuntimeError):
    pass


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def run(cmd, cwd=None, label=""):
    started = time.time()
    result = subprocess.run([str(x) for x in cmd], cwd=cwd, capture_output=True, text=True)
    record = {
        "label": label,
        "command": [str(x) for x in cmd],
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
        "duration_ms": int((time.time() - started) * 1000),
    }
    commands.append(record)
    if result.returncode != 0:
        raise AbiError("command failed: %s\n%s\n%s" % (" ".join(record["command"]),
                                                      record["stdout"], record["stderr"]))
    return record


def tool_version(exe: Path) -> str:
    result = subprocess.run([str(exe), "--version"], capture_output=True, text=True)
    if result.returncode != 0:
        return "unknown"
    lines = result.stdout.strip().splitlines()
    return lines[0] if lines else "unknown"


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def match_brace(text: str, open_index: int) -> int:
    depth = 0
    for i in range(open_index, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return i
    raise AbiError("unbalanced braces in header text")


def split_declarators(statement: str):
    parts, current, depth = [], "", 0
    for ch in statement:
        if ch in "[(":
            depth += 1
        elif ch in "])":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append(current)
            current = ""
        else:
            current += ch
    parts.append(current)
    return [p.strip() for p in parts if p.strip()]


NAME_RE = re.compile(r"([A-Za-z_]\w*)\s*((?:\[[^\]]*\]\s*)*)$")
LEAD_NAME_RE = re.compile(r"^([A-Za-z_]\w*)")


def parse_fields(body: str):
    fields = []
    for raw in body.split(";"):
        statement = " ".join(raw.split())
        if not statement:
            continue
        declarators = split_declarators(statement)
        first = declarators[0]
        m = NAME_RE.search(first)
        if not m:
            raise AbiError("cannot parse field declarator: %r" % statement)
        base_type = first[: m.start()]
        fields.append({
            "name": m.group(1),
            "type": base_type.strip(),
            "pointer": "*" in base_type,
        })
        for extra in declarators[1:]:
            m2 = LEAD_NAME_RE.match(extra)
            if not m2:
                raise AbiError("cannot parse extra declarator: %r (in %r)" % (extra, statement))
            fields.append({
                "name": m2.group(1),
                "type": base_type.strip(),
                "pointer": "*" in base_type or extra.lstrip().startswith("*"),
            })
    return fields


def parse_headers():
    types = {}
    aliases = {}
    order = []
    for header in HEADERS:
        if not header.is_file():
            raise AbiError("missing header: %s" % header)
        text = strip_comments(header.read_text(encoding="utf-8"))
        consumed = []
        for m in re.finditer(r"typedef\s+(struct|union|enum)\s*\{", text):
            kind = m.group(1)
            open_index = text.index("{", m.start())
            close_index = match_brace(text, open_index)
            tail = re.match(r"\s*([A-Za-z_]\w*)\s*;", text[close_index + 1:])
            if not tail:
                raise AbiError("cannot find typedef name in %s near offset %d" % (header.name, m.start()))
            name = tail.group(1)
            body = text[open_index + 1: close_index]
            fields = parse_fields(body) if kind in ("struct", "union") else []
            if name in types:
                raise AbiError("duplicate typedef %s" % name)
            types[name] = {"kind": kind, "fields": fields}
            order.append(name)
            consumed.append((m.start(), close_index + 1 + tail.end()))
        # Remove the consumed type definitions before looking for simple aliases.
        leftover = list(text)
        for start, end in consumed:
            for i in range(start, end):
                leftover[i] = " "
        leftover = "".join(leftover)
        for m in re.finditer(r"typedef\s+([A-Za-z_]\w*)\s+([A-Za-z_]\w*)\s*;", leftover):
            target, name = m.group(1), m.group(2)
            if name in aliases or name in types:
                raise AbiError("duplicate typedef %s" % name)
            aliases[name] = target
            order.append(name)
    return types, aliases, order


def generate_probe(types, aliases, order, lock_include: bool):
    lines = [
        "/* Generated by tests/abi/abi_report.py - do not edit. */",
        '#include "ports/port_api.h"',
        '#include "messages/message_ids.h"',
    ]
    if lock_include:
        lines.append('#include "product_contract/abi_lock.h"')
    lines += [
        "#include <stddef.h>",
        "",
        "CONTRACT_STATIC_ASSERT(CONTRACT_SCHEMA_MAJOR == %du, \"schema major\");" % EXPECTED_SCHEMA[0],
        "CONTRACT_STATIC_ASSERT(CONTRACT_SCHEMA_MINOR == %du, \"schema minor\");" % EXPECTED_SCHEMA[1],
        "CONTRACT_STATIC_ASSERT(PORT_API_REVISION == 0x%08Xu, \"port revision\");" % EXPECTED_PORT_REVISION,
        "CONTRACT_STATIC_ASSERT(PRODUCT_CONTRACT_REVISION == 0x%08Xu, \"product revision\");" % EXPECTED_PRODUCT_REVISION,
        "CONTRACT_STATIC_ASSERT(PRODUCT_PROFILE_ID == 0x%08Xu, \"profile id\");" % EXPECTED_PROFILE_ID,
    ]
    if lock_include:
        field_count = sum(len(types.get(name, {}).get("fields", [])) for name in order)
        lines += [
            "CONTRACT_STATIC_ASSERT(ABI_LOCK_PORT_API_REVISION == PORT_API_REVISION, \"lock port revision\");",
            "CONTRACT_STATIC_ASSERT(ABI_LOCK_PRODUCT_CONTRACT_REVISION == PRODUCT_CONTRACT_REVISION, \"lock product revision\");",
            "CONTRACT_STATIC_ASSERT(ABI_LOCK_CONTRACT_HEADER_BYTES == sizeof(ContractHeader), \"lock header size\");",
            f'CONTRACT_STATIC_ASSERT(ABI_LOCK_TYPE_COUNT == {len(order)}u, "lock type count");',
            f'CONTRACT_STATIC_ASSERT(ABI_LOCK_FIELD_COUNT == {field_count}u, "lock field count");',
        ]
    lines += ["", "#if defined(__GNUC__)", "#define ABI_USED __attribute__((used))",
              "#else", "#define ABI_USED", "#endif", "",
              "ABI_USED const unsigned int abi_values[] = {"]
    entries = []
    for name in order:
        lines.append("  (unsigned int)sizeof(%s)," % name)
        entries.append((name, "size"))
        lines.append("  (unsigned int)CONTRACT_ALIGNOF(%s)," % name)
        entries.append((name, "alignment"))
        for field in types.get(name, {}).get("fields", []):
            lines.append("  (unsigned int)offsetof(%s, %s)," % (name, field["name"]))
            entries.append((name, "offset:" + field["name"]))
    lines += ["};", ""]
    return "\n".join(lines), entries


def dump_values(obj: Path, objcopy: Path, expected: int, preferred: str):
    last_error = None
    tried = []
    for section in dict.fromkeys([preferred, ".rodata", ".rdata", ".text"]):
        tried.append(section)
        with tempfile.NamedTemporaryFile(delete=False, suffix=".bin") as handle:
            binary = Path(handle.name)
        try:
            result = subprocess.run(
                [str(objcopy), "--dump-section", "%s=%s" % (section, binary), str(obj)],
                capture_output=True, text=True)
            if result.returncode != 0:
                last_error = "%s: %s" % (section, result.stderr.strip())
                continue
            raw = binary.read_bytes()
            if len(raw) < expected * 4:
                last_error = "section %s size %d < %d" % (section, len(raw), expected * 4)
                continue
            padding = raw[expected * 4:]
            if any(padding):
                last_error = "section %s has %d non-zero trailing bytes" % (section, len(padding))
                continue
            commands.append({
                "label": "objcopy %s" % section,
                "command": [str(objcopy), "--dump-section", "%s=%s" % (section, binary), str(obj)],
                "returncode": 0,
                "stdout": result.stdout.strip(),
                "stderr": result.stderr.strip(),
                "duration_ms": 0,
            })
            return list(struct.unpack("<%dI" % expected, raw[: expected * 4])), section
        finally:
            binary.unlink(missing_ok=True)
    raise AbiError("cannot locate %d ABI words in %s (tried %s; %s)" %
                   (expected, obj.name, ",".join(tried), last_error))


def build_layout(types, aliases, order, entries, values):
    if len(values) != len(entries):
        raise AbiError("value count mismatch")
    layout = {}
    for name in order:
        layout[name] = {"kind": aliases.get(name, types.get(name, {}).get("kind", "alias")),
                        "fields": {}}
    for (name, kind), value in zip(entries, values):
        if kind == "size":
            layout[name]["size"] = value
        elif kind == "alignment":
            layout[name]["alignment"] = value
        else:
            layout[name]["fields"][kind.split(":", 1)[1]] = value
    return layout


def canonical_abi(layout, target: str, pointer_bits: int):
    lines = [
        "FC-ABI 1",
        "target %s" % target,
        "pointer_bits %d" % pointer_bits,
        "profile 0x%08X" % EXPECTED_PROFILE_ID,
        "schema %d.%d" % EXPECTED_SCHEMA,
        "port_revision 0x%08X" % EXPECTED_PORT_REVISION,
        "product_revision 0x%08X" % EXPECTED_PRODUCT_REVISION,
    ]
    for name in sorted(layout):
        item = layout[name]
        lines.append("T %s %d %d %s" % (name, item["size"], item["alignment"], item["kind"]))
        for field in sorted(item["fields"]):
            lines.append("F %s.%s %d" % (name, field, item["fields"][field]))
    return "\n".join(lines) + "\n"


def pointer_bearing(types, aliases):
    pointing = {name for name, info in types.items()
                if any(f["pointer"] for f in info["fields"])}
    changed = True
    while changed:
        changed = False
        for name, target in aliases.items():
            if target in pointing and name not in pointing:
                pointing.add(name)
                changed = True
    return pointing


def compare_with_spec(arm_layout):
    spec = json.loads(SPEC_LAYOUT.read_text(encoding="utf-8"))
    current_icd_sha = sha256_file(ICD)
    if spec.get("icd_sha256") != current_icd_sha:
        return {
            "spec_file": str(SPEC_LAYOUT.relative_to(ROOT)).replace("\\", "/"),
            "spec_sha256": sha256_file(SPEC_LAYOUT),
            "spec_icd_sha256": spec.get("icd_sha256"),
            "current_icd_sha256": current_icd_sha,
            "spec_compiler": spec.get("compiler"),
            "compared_types": 0,
            "compared_field_offsets": 0,
            "mismatches": [],
            "missing_types": [],
            "extra_types": [],
            "excluded_private_types": sorted(PRIVATE_TYPES & set(spec["layouts"])),
            "status": "STALE_BASELINE",
            "reason": "Regenerate spec/validation/abi_layout.json from the current ICD "
                      "with spec/tools/audit_spec.py before verifying or emitting the lock.",
        }
    spec_layouts = spec["layouts"]
    mismatches = []
    compared = 0
    offsets_compared = 0
    missing = []
    extra = []
    for name, expected in spec_layouts.items():
        if name in PRIVATE_TYPES:
            continue
        if name not in arm_layout:
            missing.append(name)
            continue
        compared += 1
        actual = arm_layout[name]
        if actual["size"] != expected["size"]:
            mismatches.append({"type": name, "kind": "size",
                               "expected": expected["size"], "actual": actual["size"]})
        if actual["alignment"] != expected["alignment"]:
            mismatches.append({"type": name, "kind": "alignment",
                               "expected": expected["alignment"], "actual": actual["alignment"]})
        for key, value in expected.items():
            if not key.startswith("offset:"):
                continue
            offsets_compared += 1
            field = key.split(":", 1)[1]
            if field not in actual["fields"]:
                mismatches.append({"type": name, "kind": "missing_field", "field": field})
            elif actual["fields"][field] != value:
                mismatches.append({"type": name, "kind": "offset", "field": field,
                                   "expected": value, "actual": actual["fields"][field]})
    for name in arm_layout:
        if name not in spec_layouts:
            extra.append(name)
    return {
        "spec_file": str(SPEC_LAYOUT.relative_to(ROOT)).replace("\\", "/"),
        "spec_sha256": sha256_file(SPEC_LAYOUT),
        "spec_compiler": spec.get("compiler"),
        "compared_types": compared,
        "compared_field_offsets": offsets_compared,
        "excluded_private_types": sorted(PRIVATE_TYPES & set(spec_layouts)),
        "missing_types": missing,
        "extra_types": extra,
        "mismatches": mismatches,
        "status": "PASS" if not (missing or extra or mismatches) else "FAIL",
    }


def compare_targets(arm_layout, host_layout, pointing):
    identical, differing, unexpected = 0, [], []
    for name in sorted(arm_layout):
        arm, host = arm_layout[name], host_layout[name]
        same = (arm["size"] == host["size"] and arm["alignment"] == host["alignment"]
                and arm["fields"] == host["fields"])
        if same:
            identical += 1
        else:
            field_differences = [
                field for field in sorted(set(arm["fields"]) | set(host["fields"]))
                if arm["fields"].get(field) != host["fields"].get(field)
            ]
            differing.append({
                "type": name,
                "arm_size": arm["size"], "host_size": host["size"],
                "field_differences": field_differences,
                "pointer_bearing": name in pointing,
            })
            if name not in pointing:
                unexpected.append(name)
    return {
        "identical_types": identical,
        "differing_types": differing,
        "unexpected_differences": unexpected,
        "status": "PASS" if not unexpected else "FAIL",
    }


def camel_to_snake(name: str) -> str:
    step = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    step = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", step)
    return step.upper()


def check_message_registry(header_path: Path):
    text = strip_comments(header_path.read_text(encoding="utf-8"))
    macros = {}
    for m in re.finditer(r"^#define\s+(MSG_[A-Z0-9_]+)\s+(0x[0-9A-Fa-f]+|\d+)u?\b", text, re.M):
        macros[m.group(1)] = int(m.group(2), 0)
    registry = json.loads(MESSAGE_REGISTRY.read_text(encoding="utf-8"))
    problems = []
    expected = {}
    for message, value in registry["messages"].items():
        macro = "MSG_" + camel_to_snake(message)
        expected[macro] = value
        if macro not in macros:
            problems.append({"issue": "missing_macro", "message": message, "macro": macro})
        elif macros[macro] != value:
            problems.append({"issue": "value_mismatch", "macro": macro,
                             "expected": value, "actual": macros[macro]})
    for macro, value in macros.items():
        if macro in expected:
            continue
        alias_target = macro.replace("MSG_FMS_HEALTH", "MSG_MODULE_HEALTH") \
                            .replace("MSG_GUIDANCE_HEALTH", "MSG_MODULE_HEALTH") \
                            .replace("MSG_ACTUATOR_HEALTH", "MSG_MODULE_HEALTH")
        if alias_target in macros and macros[alias_target] == value:
            continue
        problems.append({"issue": "unregistered_macro", "macro": macro, "value": value})
    return {
        "registry_file": str(MESSAGE_REGISTRY.relative_to(ROOT)).replace("\\", "/"),
        "registry_sha256": sha256_file(MESSAGE_REGISTRY),
        "registered_messages": len(registry["messages"]),
        "macros_checked": len(macros),
        "problems": problems,
        "status": "PASS" if not problems else "FAIL",
    }


def check_firmware_flags():
    """The firmware build must use the same layout-relevant flags as this ABI
    probe; GNU Arm defaults to short enums, which breaks the 32-bit public
    status enums.  This check is part of the ABI evidence (D-REVIEW B-R03)."""
    config = (ROOT / "rtconfig.py").read_text(encoding="utf-8")
    required = ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16",
                "-mfloat-abi=hard", "-fno-short-enums"]
    missing = [flag for flag in required if flag not in config]
    return {
        "file": "rtconfig.py",
        "required_flags": required,
        "missing": missing,
        "status": "PASS" if not missing else "FAIL",
    }


def write_lock(arm_hash, host_hash, arm_layout, type_count, field_count, header_sha, registry_sha):
    text = f"""/*
 * GENERATED FILE - do not edit.
 *
 * QUAD_X4 Schema 6.0 ABI lock for baseline {BASELINE}.
 * Producer: tests/abi/abi_report.py --emit-lock
 * Canonical input SHA256 (production headers):
 *   {header_sha}
 *
 * Any change to a public type size/alignment/field offset, or to the message
 * type registry, changes these values.  Regeneration is an ICD change
 * requiring a schema/product revision review (spec 02 section 4.15); CI must
 * run the tool without --emit-lock and fail on drift.
 */
#ifndef ABI_LOCK_H
#define ABI_LOCK_H

#define ABI_LOCK_BASELINE "{BASELINE}"
#define ABI_LOCK_MODEL {1}u
#define ABI_LOCK_SCHEMA_MAJOR {EXPECTED_SCHEMA[0]}u
#define ABI_LOCK_SCHEMA_MINOR {EXPECTED_SCHEMA[1]}u
#define ABI_LOCK_PORT_API_REVISION 0x{EXPECTED_PORT_REVISION:08X}u
#define ABI_LOCK_PRODUCT_CONTRACT_REVISION 0x{EXPECTED_PRODUCT_REVISION:08X}u
#define ABI_LOCK_PROFILE_ID 0x{EXPECTED_PROFILE_ID:08X}u
#define ABI_LOCK_CONTRACT_HEADER_BYTES {arm_layout['ContractHeader']['size']}u
#define ABI_LOCK_TYPE_COUNT {type_count}u
#define ABI_LOCK_FIELD_COUNT {field_count}u
#define ABI_LOCK_MESSAGE_REGISTRY_SHA256 "{registry_sha}"
#define ABI_LOCK_ARM_HASH "{arm_hash}"
#define ABI_LOCK_HOST_HASH "{host_hash}"

#endif /* ABI_LOCK_H */
"""
    LOCK_FILE.write_text(text, encoding="utf-8", newline="\n")


def check_lock(arm_hash, host_hash, registry_sha):
    if not LOCK_FILE.is_file():
        return {"status": "FAIL", "reason": "abi_lock.h missing; run with --emit-lock"}
    text = LOCK_FILE.read_text(encoding="utf-8")
    arm = re.search(r'ABI_LOCK_ARM_HASH\s+"([0-9a-f]{64})"', text)
    host = re.search(r'ABI_LOCK_HOST_HASH\s+"([0-9a-f]{64})"', text)
    registry = re.search(r'ABI_LOCK_MESSAGE_REGISTRY_SHA256\s+"([0-9a-f]{64})"', text)
    problems = []
    if not arm or arm.group(1) != arm_hash:
        problems.append({"hash": "arm", "locked": arm.group(1) if arm else None, "actual": arm_hash})
    if not host or host.group(1) != host_hash:
        problems.append({"hash": "host", "locked": host.group(1) if host else None, "actual": host_hash})
    if not registry or registry.group(1) != registry_sha:
        problems.append({"hash": "message_registry",
                         "locked": registry.group(1) if registry else None,
                         "actual": registry_sha})
    return {"status": "PASS" if not problems else "FAIL", "problems": problems,
            "lock_file": str(LOCK_FILE.relative_to(ROOT)).replace("\\", "/"),
            "lock_sha256": sha256_file(LOCK_FILE)}


def check_header_hygiene(types):
    """Public headers must be self-contained and free of platform includes."""
    allowed_includes = {"stdint.h", "stddef.h"}
    allowed_includes.update(header.relative_to(CONTRACTS).as_posix()
                            for header in HEADERS if header.is_relative_to(CONTRACTS))
    problems = []
    for header in HEADERS:
        text = strip_comments(header.read_text(encoding="utf-8"))
        for include in re.findall(r'#include\s+[<"]([^>"]+)[>"]', text):
            if include not in allowed_includes:
                problems.append({"file": header.name, "include": include})
        if re.search(r"\bvoid\s*\*", text):
            problems.append({"file": header.name, "issue": "void pointer"})

    registry = json.loads(MESSAGE_REGISTRY.read_text(encoding="utf-8"))
    message_types = set(registry["messages"]) | set(registry["aliases"])
    pointer_messages = []
    for name in sorted(message_types):
        info = types.get(name)
        if not info:
            continue
        if any(field["pointer"] for field in info["fields"]):
            pointer_messages.append(name)
    return {
        "headers_checked": len(HEADERS),
        "forbidden_includes_or_void_pointers": problems,
        "messages_with_pointer_fields": pointer_messages,
        "status": "PASS" if not problems and not pointer_messages else "FAIL",
    }


def check_self_contained(arm_cc, host_cxx, include_flags):
    """Compile each public declaration header on its own."""
    failures = []
    with tempfile.TemporaryDirectory(prefix="flight-abi-sc-") as tmp_name:
        tmp = Path(tmp_name)
        for header in HEADERS:
            src = tmp / ("sc_%s.c" % header.stem)
            src.write_text('#include "%s"\nint sc_anchor(void);\n' % header.relative_to(CONTRACTS).as_posix(),
                           encoding="utf-8", newline="\n")
            obj = tmp / ("sc_%s.o" % header.stem)
            for label, compiler, flags in (
                ("arm-c99", arm_cc, ["-std=c99"] + ARM_FLAGS),
                ("host-c++14", host_cxx, ["-std=c++14", "-fno-exceptions", "-fno-rtti"] + HOST_FLAGS),
            ):
                try:
                    run([compiler] + flags + include_flags + ["-c", src, "-o", obj],
                        label="self-contained %s %s" % (header.name, label))
                except AbiError as exc:
                    failures.append({"header": header.name, "mode": label, "error": str(exc)})
    return {
        "headers_checked": len(HEADERS),
        "failures": failures,
        "status": "PASS" if not failures else "FAIL",
    }


def write_json(path: Path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def probe_tools(directory, prefix, option):
    suffix = ".exe" if os.name == "nt" else ""
    tools = []
    for executable in ("gcc", "g++", "objcopy"):
        name = prefix + executable + suffix
        path = Path(directory) / name if directory else shutil.which(name)
        if not path or not Path(path).is_file():
            raise AbiError(f"toolchain not found: {path or name} (use {option})")
        tools.append(Path(path).absolute())
    return tools


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--emit-lock", action="store_true",
                        help="regenerate middleware/contracts/product_contract/abi_lock.h")
    parser.add_argument("--arm-bin", default=os.environ.get("FLIGHT_ARM_BIN"))
    parser.add_argument("--host-bin", default=os.environ.get("FLIGHT_HOST_BIN"))
    args = parser.parse_args()

    arm_bin = args.arm_bin or (DEFAULT_ARM_BIN if DEFAULT_ARM_BIN.is_dir() else None)
    host_bin = args.host_bin or (DEFAULT_HOST_BIN if DEFAULT_HOST_BIN.is_dir() else None)
    host_prefix = "" if os.name == "nt" else "x86_64-w64-mingw32-"
    arm_cc, arm_cxx, arm_objcopy = probe_tools(arm_bin, "arm-none-eabi-", "--arm-bin")
    host_cc, host_cxx, host_objcopy = probe_tools(host_bin, host_prefix, "--host-bin")

    EVIDENCE.mkdir(parents=True, exist_ok=True)
    types, aliases, order = parse_headers()
    include_flags = ["-I" + str(CONTRACTS)]
    targets = {}
    pointer_bits = {"arm": 32, "host": 64}
    target_names = {"arm": "arm-none-eabi-cortex-m7-eabihf", "host": "x86_64-w64-mingw32"}

    def measure(probe_text, entries):
        """Compile the probe for ARM and Host in C11 and C++14, extract values."""
        nonlocal targets
        targets = {}
        values_by_target = {}
        with tempfile.TemporaryDirectory(prefix="flight-abi-") as tmp_name:
            tmp = Path(tmp_name)
            src = tmp / "abi_probe.c"
            src.write_text(probe_text, encoding="utf-8", newline="\n")
            variants = [
                ("arm", "c11", arm_cc, ["-std=c11"] + ARM_FLAGS),
                ("arm", "c++14", arm_cxx,
                 ["-std=c++14", "-fno-exceptions", "-fno-rtti"] + ARM_FLAGS),
                ("host", "c11", host_cc, ["-std=c11"] + HOST_FLAGS),
                ("host", "c++14", host_cxx,
                 ["-std=c++14", "-fno-exceptions", "-fno-rtti"] + HOST_FLAGS),
            ]
            for target, language, compiler, flags in variants:
                obj = tmp / ("%s_%s.o" % (target, language.replace("+", "p")))
                run([compiler] + flags + include_flags + ["-c", src, "-o", obj],
                    label="compile %s %s" % (target, language))
                objcopy = arm_objcopy if target == "arm" else host_objcopy
                preferred = ".rodata" if target == "arm" else ".rdata"
                values, section = dump_values(obj, objcopy, len(entries), preferred)
                values_by_target[(target, language)] = values
                targets.setdefault(target, {})[language] = {
                    "compiler": str(compiler),
                    "compiler_version": tool_version(compiler),
                    "flags": flags + include_flags,
                    "object_section": section,
                    "values_sha256": sha256_bytes(struct.pack("<%dI" % len(values), *values)),
                    "types": len(order),
                    "entries": len(entries),
                }
                if language == "c++14":
                    if values != values_by_target[(target, "c11")]:
                        raise AbiError("%s C and C++ layouts differ" % target)
        return values_by_target

    def layout_and_hash(values_by_target, entries):
        layouts = {}
        for target in ("arm", "host"):
            layouts[target] = build_layout(types, aliases, order, entries,
                                           values_by_target[(target, "c11")])
        hashes = {}
        for target in ("arm", "host"):
            canonical = canonical_abi(layouts[target], target_names[target], pointer_bits[target])
            hashes[target] = sha256_bytes(canonical.encode("utf-8"))
            targets[target]["abi"] = target_names[target]
            targets[target]["pointer_bits"] = pointer_bits[target]
            targets[target]["abi_hash"] = hashes[target]
            targets[target]["layout"] = layouts[target]
        return layouts, hashes

    # First pass avoids a lock dependency so a fresh checkout can bootstrap.
    probe_plain, entries = generate_probe(types, aliases, order, False)
    values_by_target = measure(probe_plain, entries)
    layouts, hashes = layout_and_hash(values_by_target, entries)

    if args.emit_lock:
        spec_cmp = compare_with_spec(layouts["arm"])
        if spec_cmp["status"] != "PASS":
            raise AbiError("lock generation requires a matching spec layout: " + json.dumps(spec_cmp))
        field_count = sum(len(item["fields"]) for item in layouts["arm"].values())
        header_sha = sha256_bytes("\n".join(
            "%s\0%s" % (h.relative_to(ROOT).as_posix(), sha256_file(h)) for h in HEADERS
        ).encode("utf-8"))
        write_lock(hashes["arm"], hashes["host"], layouts["arm"], len(order),
                   field_count, header_sha, sha256_file(MESSAGE_REGISTRY))

    # Second pass: compile with the lock's static assertions when it exists, so
    # the recorded hashes are enforced by the compiler as well as by this tool.
    final_probe = probe_plain
    if LOCK_FILE.is_file():
        probe_locked, entries_locked = generate_probe(types, aliases, order, True)
        if entries_locked != entries:
            raise AbiError("lock-enabled probe entry list changed")
        values_by_target = measure(probe_locked, entries)
        layouts, hashes = layout_and_hash(values_by_target, entries)
        final_probe = probe_locked

    # Keep the exact probe and canonical hash inputs with the evidence so a
    # reviewer can recompute the hashes with ordinary tools.
    (EVIDENCE / "abi_probe.c").write_text(final_probe, encoding="utf-8", newline="\n")
    for target in ("arm", "host"):
        canonical = canonical_abi(layouts[target], target_names[target], pointer_bits[target])
        actual = sha256_bytes(canonical.encode("utf-8"))
        if actual != hashes[target]:
            raise AbiError("%s canonical hash mismatch" % target)
        (EVIDENCE / ("abi_canonical_%s.txt" % target)).write_text(
            canonical, encoding="utf-8", newline="\n")

    lock = check_lock(hashes["arm"], hashes["host"], sha256_file(MESSAGE_REGISTRY))
    spec_cmp = compare_with_spec(layouts["arm"])
    target_cmp = compare_targets(layouts["arm"], layouts["host"], pointer_bearing(types, aliases))
    registry = check_message_registry(CONTRACTS / "messages/message_ids.h")
    hygiene = check_header_hygiene(types)
    self_contained = check_self_contained(arm_cc, host_cxx, include_flags)
    firmware_flags = check_firmware_flags()

    failures = [name for name, item in (
        ("spec_layout", spec_cmp), ("host_vs_arm", target_cmp),
        ("message_registry", registry), ("abi_lock", lock),
        ("header_hygiene", hygiene), ("header_self_containment", self_contained),
        ("firmware_flags", firmware_flags),
    ) if item["status"] != "PASS"]
    warnings = []

    evidence_common = {
        "baseline": BASELINE,
        "scope": "HOST_PRODUCTION_ABI",
        "independent_review": "NOT_RUN",
        "target_firmware_build": "NOT_RUN",
        "profile": "QUAD_X4",
        "schema": "6.0",
        "port_api_revision": "0x%08X" % EXPECTED_PORT_REVISION,
        "product_contract_revision": "0x%08X" % EXPECTED_PRODUCT_REVISION,
        "headers": [str(h.relative_to(ROOT)).replace("\\", "/") for h in HEADERS],
        "header_sha256": {str(h.relative_to(ROOT)).replace("\\", "/"): sha256_file(h) for h in HEADERS},
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "warnings": warnings,
    }

    arm_report = dict(evidence_common)
    arm_report.update({"compiler": targets["arm"]["c11"]["compiler_version"],
                       "flags": targets["arm"]["c11"]["flags"],
                       "abi_hash": hashes["arm"], "types": len(order),
                       "fields": sum(len(item["fields"]) for item in layouts["arm"].values()),
                       "layout": layouts["arm"]})
    host_report = dict(evidence_common)
    host_report.update({"compiler": targets["host"]["c11"]["compiler_version"],
                        "flags": targets["host"]["c11"]["flags"],
                        "abi_hash": hashes["host"], "types": len(order),
                        "fields": sum(len(item["fields"]) for item in layouts["host"].values()),
                        "layout": layouts["host"]})
    compare_report = {
        "baseline": BASELINE,
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "abi_hashes": hashes,
        "spec_layout_comparison": spec_cmp,
        "warnings": warnings,
        "host_arm_comparison": target_cmp,
        "message_registry_check": registry,
        "header_hygiene": hygiene,
        "header_self_containment": self_contained,
        "firmware_build_flags": firmware_flags,
        "abi_lock_check": lock,
        "private_types_excluded": sorted(PRIVATE_TYPES),
    }

    write_json(EVIDENCE / "abi_arm.json", arm_report)
    write_json(EVIDENCE / "abi_host.json", host_report)
    write_json(EVIDENCE / "abi_compare.json", compare_report)
    write_json(EVIDENCE / "commands.json", {
        "baseline": BASELINE,
        "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "commands": commands,
    })

    def table(headers, rows):
        out = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
        out += ["| " + " | ".join(str(c) for c in row) + " |" for row in rows]
        return "\n".join(out)

    key_sizes = [
        "ContractHeader", "NavigationSnapshot", "MotionReference", "ControllerInput",
        "FmsInputView", "FmsOutputView", "ActuatorDemand", "ProductMissionPayload",
        "AuthorizedMissionSnapshot", "ProductEstimatorConfigPayload",
    ]
    key_rows = [(name, layouts["arm"][name]["size"], layouts["host"][name]["size"],
                 layouts["arm"][name]["alignment"]) for name in key_sizes]
    diff_rows = [(item["type"], item["arm_size"], item["host_size"],
                  len(item["field_differences"])) for item in target_cmp["differing_types"]]
    report = f"""# T191-A-ABI Host/ARM production ABI evidence

- Baseline: `{BASELINE}` / Schema 6.0 / Port `0x{EXPECTED_PORT_REVISION:08X}` / Product `0x{EXPECTED_PRODUCT_REVISION:08X}`
- Profile: QUAD_X4 (`0x{EXPECTED_PROFILE_ID:08X}`)
- Scope: `HOST_PRODUCTION_ABI` (production headers compiled in C11 and C++14 for ARM Cortex-M7 and x86_64 host)
- Result: **{'PASS' if not failures else 'FAIL'}**{'' if not failures else ' — ' + ', '.join(failures)}
- Independent review: NOT_RUN (reviewer D)
- Target firmware build, consumer recompilation, WCET, HIL and flight: NOT_RUN

## ABI hashes

| Target | Pointer bits | ABI hash |
|---|---|---|
| ARM Cortex-M7 EABI hard-float | 32 | `{hashes['arm']}` |
| x86_64 MinGW Host | 64 | `{hashes['host']}` |

Canonical ABI text is `TYPE/FIELD` ordered by type name and field name; see
`tests/abi/abi_report.py::canonical_abi`.

## Types and fields

- Public types probed: **{len(order)}**
- Public fields probed: **{sum(len(item['fields']) for item in layouts['arm'].values())}**
- Private types excluded: {', '.join('`%s`' % x for x in sorted(PRIVATE_TYPES))}

## Key sizes (ARM / Host, bytes)

{table(['Type', 'ARM', 'Host', 'ARM align'], key_rows)}

## Cross-check against spec/validation/abi_layout.json

- Status: **{spec_cmp['status']}**
- Compared types: **{spec_cmp['compared_types']}**
- Compared field offsets: **{spec_cmp['compared_field_offsets']}**
- Mismatches: **{len(spec_cmp['mismatches'])}**
- Missing types: **{len(spec_cmp['missing_types'])}**
- Extra types: **{len(spec_cmp['extra_types'])}**
- Spec compiler: {spec_cmp['spec_compiler']}
{('- Reason: ' + spec_cmp['reason']) if spec_cmp.get('reason') else ''}

## Host vs ARM layout delta

- Identical layouts: **{target_cmp['identical_types']}**
- Pointer-dependent layouts (expected): **{len(target_cmp['differing_types'])}**
- Unexpected differences: **{len(target_cmp['unexpected_differences'])}**

{table(['Type', 'ARM size', 'Host size', 'Differing fields'], diff_rows)}

`FmsInputSnapshot` is a typedef alias of `FmsInputView` and inherits its layout
without independent fields. Pointer-bearing Port views are local to the calling
process and have separate ARM and Host layouts.

## Message registry

- Registry entries: **{registry['registered_messages']}**, macros checked: **{registry['macros_checked']}**
- Problems: **{len(registry['problems'])}**

## Header hygiene and self-containment

- Headers checked for forbidden includes / `void*`: **{hygiene['headers_checked']}**
  (problems: **{len(hygiene['forbidden_includes_or_void_pointers'])}**)
- Registered messages carrying pointer fields: **{len(hygiene['messages_with_pointer_fields'])}**
- Headers compiled standalone (ARM C99 + Host C++14): **{self_contained['headers_checked']}**
  (failures: **{len(self_contained['failures'])}**)

## Firmware build flags

- `rtconfig.py` layout flags required: {', '.join('`%s`' % f for f in firmware_flags['required_flags'])}
- Missing: **{len(firmware_flags['missing'])}** (GNU Arm defaults to short enums; the public ICD stores 32-bit status enums)

## ABI lock

- Lock file: `middleware/contracts/product_contract/abi_lock.h`
- Status: **{lock['status']}**
- Message registry SHA256 bound by the lock: `{registry['registry_sha256']}`

## Artifacts

- `tests/abi/evidence/abi_arm.json`
- `tests/abi/evidence/abi_host.json`
- `tests/abi/evidence/abi_compare.json`
- `tests/abi/evidence/commands.json`
- `tests/abi/evidence/abi_probe.c` (exact compiled probe)
- `tests/abi/evidence/abi_canonical_arm.txt` / `abi_canonical_host.txt`
  (their SHA256 equals the ABI hashes above)

## What this evidence does not prove

Compiling the headers and matching the normative layout does not prove a
target firmware build, consumer recompilation, static-memory/WCET budget,
target register/waveform behaviour, SIL/HIL or flight safety.  Those remain
NOT_RUN in this report. See spec 09 for system and flight evidence.
"""
    (EVIDENCE / "abi_report.md").write_text(report, encoding="utf-8", newline="\n")

    manifest_path = EVIDENCE / "artifact_sha256.json"
    artifacts = sorted(
        path for root in (CONTRACTS, HERE)
        for path in root.rglob("*")
        if path.is_file() and path != manifest_path and "__pycache__" not in path.parts
    )
    write_json(manifest_path, {
        "baseline": BASELINE,
        "algorithm": "sha256",
        "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "file_count": len(artifacts),
        "files": {str(path.relative_to(ROOT)).replace("\\", "/"): sha256_file(path)
                  for path in artifacts},
    })

    summary = {
        "status": "PASS" if not failures else "FAIL",
        "types": len(order),
        "arm_hash": hashes["arm"],
        "host_hash": hashes["host"],
        "spec_mismatches": len(spec_cmp["mismatches"]),
        "unexpected_host_arm": len(target_cmp["unexpected_differences"]),
        "lock": lock["status"],
        "evidence": str(EVIDENCE.relative_to(ROOT)).replace("\\", "/"),
    }
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AbiError as exc:
        print("ABI-ERROR: %s" % exc, file=sys.stderr)
        sys.exit(2)
