"""Git blob identity and fail-closed refresh for embedded OSQP source locks."""
import hashlib
import json
from pathlib import Path


def git_blob_sha1(content: bytes) -> str:
    """Return the Git object ID for a blob, including its NUL-terminated header."""
    header = b"blob " + str(len(content)).encode("ascii") + bytes((0,))
    return hashlib.sha1(header + content).hexdigest()


def refresh_lock(lock_path: Path, generated_root: Path) -> dict:
    """Refresh generated/local-patch hashes while preserving upstream provenance.

    The lock's file set must exactly match the embedded generated tree, excluding
    explicitly declared host-only files and compiler by-products. Files marked
    upstream are immutable: their bytes must still equal the pinned upstream blob.
    """
    lock_path = Path(lock_path)
    generated_root = Path(generated_root)
    lock = json.loads(lock_path.read_text(encoding="utf-8"))

    if lock.get("schema_version") != 2 or lock.get("hash_algorithm") != "git-blob-sha1":
        raise ValueError("Unsupported OSQP source-lock schema or hash algorithm")

    files = lock.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("OSQP source lock must contain a non-empty files mapping")

    host_only = lock.get("host_only_files", [])
    if not isinstance(host_only, list) or any(not isinstance(item, str) for item in host_only):
        raise ValueError("host_only_files must be a list of relative paths")
    host_only_set = set(host_only)

    actual_paths = set()
    for path in generated_root.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(generated_root).as_posix()
        if rel in host_only_set or path.suffix in {".o", ".su"}:
            continue
        actual_paths.add(rel)

    locked_paths = set(files)
    if actual_paths != locked_paths:
        missing = sorted(locked_paths - actual_paths)
        unexpected = sorted(actual_paths - locked_paths)
        raise ValueError(
            f"OSQP embedded file set mismatch; missing={missing}, unexpected={unexpected}"
        )

    for rel, entry in sorted(files.items()):
        rel_path = Path(rel)
        if rel_path.is_absolute() or ".." in rel_path.parts:
            raise ValueError(f"Invalid path in OSQP source lock: {rel}")
        if not isinstance(entry, dict):
            raise ValueError(f"Invalid metadata for OSQP source: {rel}")

        digest = git_blob_sha1((generated_root / rel_path).read_bytes())
        provenance = entry.get("provenance")

        if provenance == "upstream":
            expected = entry.get("upstream", {}).get("blob_sha1")
            if not expected or digest != expected:
                raise ValueError(
                    f"Upstream OSQP file differs from pinned source: {rel}; "
                    f"expected={expected}, actual={digest}"
                )
        elif provenance == "local_patch":
            upstream = entry.get("upstream", {})
            if not upstream.get("blob_sha1") or not entry.get("patch_summary"):
                raise ValueError(f"Incomplete local patch provenance: {rel}")
        elif provenance == "generated":
            if not entry.get("generator"):
                raise ValueError(f"Missing generator provenance: {rel}")
        else:
            raise ValueError(f"Unknown OSQP provenance for {rel}: {provenance!r}")

        entry["git_blob_sha1"] = digest

    temporary = lock_path.with_name(lock_path.name + ".tmp")
    temporary.write_text(json.dumps(lock, indent=2) + "\n", encoding="utf-8")
    temporary.replace(lock_path)
    return lock
