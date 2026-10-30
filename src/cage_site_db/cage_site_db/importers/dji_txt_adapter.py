import os
import resource
import subprocess
import tempfile

from .. import config
from ..assets import disk_has_room, immutable_copy, sha256_file
from ..db import dump, now, row, transaction, uid
from .csv_importer import import_csv


def _limits():
    resource.setrlimit(resource.RLIMIT_CPU, (120, 120))
    resource.setrlimit(resource.RLIMIT_AS, (1024 * 1024 * 1024, 1024 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_FSIZE, (512 * 1024 * 1024, 512 * 1024 * 1024))


def inspect_txt(path):
    if not os.path.isfile(path):
        return {"ok": False, "error_code": "DJI_TXT_PENDING_UPLOAD"}
    size = os.path.getsize(path)
    if size < 12:
        return {"ok": False, "error_code": "DJI_TXT_INVALID_FORMAT", "detail": "file shorter than DJI prefix"}
    with open(path, "rb") as handle:
        head = handle.read(4096)
    text_score = sum(1 for byte in head if byte in b"\r\n\t" or 32 <= byte <= 126) / max(1, len(head))
    if text_score > 0.92 and (b"," in head or b"latitude" in head.lower()):
        return {"ok": False, "error_code": "DJI_TXT_INVALID_FORMAT", "detail": "text/CSV is not a raw DJI FlightRecord"}
    version = head[10]
    if version < 1 or version > 50:
        return {"ok": False, "error_code": "DJI_TXT_UNSUPPORTED_VERSION", "version": version}
    parser_ready = os.path.isfile(config.PARSER_BIN) and os.access(config.PARSER_BIN, os.X_OK)
    result = {"ok": parser_ready, "version": version, "encrypted": version >= 13,
              "parser_name": config.PARSER_NAME, "parser_version": config.PARSER_VERSION,
              "parser_installed": parser_ready}
    if not parser_ready:
        result.update(ok=False, error_code="DJI_TXT_PARSER_NOT_INSTALLED")
    elif version >= 13 and not os.environ.get("DJI_LOG_API_KEY"):
        result.update(ok=False, error_code="DJI_TXT_DECRYPTION_KEY_REQUIRED")
    elif version >= 13 and os.environ.get("DJI_ALLOW_KEYCHAIN_NETWORK") != "1":
        result.update(ok=False, error_code="DJI_TXT_DECRYPTION_UNAVAILABLE",
                      detail="key present but external DJI keychain request is not explicitly enabled")
    return result


def import_txt(path, original_name, site_id=None, source_role="DJI_FLY_RAW_EXPORT"):
    size = os.path.getsize(path)
    if size > config.MAX_TXT_BYTES:
        raise ValueError("DJI_TXT_TOO_LARGE")
    if not disk_has_room(config.ROOT, size):
        raise ValueError("DISK_RESERVE_REACHED")
    digest = sha256_file(path)
    with transaction() as conn:
        prior = row(conn.execute("SELECT * FROM source_assets WHERE sha256=?", (digest,)))
        if prior:
            prior["idempotent"] = True
            return prior
    inspection = inspect_txt(path)
    target = immutable_copy(path, digest, ".txt")
    asset_id = uid("asset")
    with transaction() as conn:
        conn.execute("INSERT INTO source_assets VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", (
            asset_id, digest, "DJI_TXT", original_name, target, source_role, size, "application/octet-stream",
            config.PARSER_NAME, config.PARSER_VERSION, "INSPECTED" if inspection.get("ok") else "BLOCKED",
            inspection.get("error_code"), inspection.get("detail"), dump(inspection), now()))
    if not inspection.get("ok"):
        return {"id": asset_id, "sha256": digest, "import_status": "BLOCKED", **inspection}
    with tempfile.TemporaryDirectory(prefix="dji_txt_", dir=config.NORMALIZED) as temp:
        csv_path = os.path.join(temp, "frames.csv")
        command = [config.PARSER_BIN, target, "--csv", csv_path]
        key = os.environ.get("DJI_LOG_API_KEY")
        if key:
            command.extend(["--api-key", key])
        try:
            proc = subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=150,
                                  check=False, preexec_fn=_limits)
        except subprocess.TimeoutExpired:
            code, detail = "DJI_TXT_PARSE_FAILED", "parser timeout"
        else:
            code, detail = (None, None) if proc.returncode == 0 and os.path.isfile(csv_path) else (
                "DJI_TXT_PARSE_FAILED", proc.stderr.decode("utf-8", "replace")[-1000:])
        if code:
            with transaction() as conn:
                conn.execute("UPDATE source_assets SET import_status='FAILED',error_code=?,error_detail=? WHERE id=?",
                             (code, detail, asset_id))
            return {"id": asset_id, "sha256": digest, "import_status": "FAILED", "error_code": code, "detail": detail}
        result = import_csv(csv_path, original_name + ".normalized.csv", site_id=site_id, source_role=source_role,
                            parser=config.PARSER_NAME, parser_version=config.PARSER_VERSION,
                            existing_asset={"id": asset_id, "original_path": target}, origin_type="DJI_TXT")
        with transaction() as conn:
            conn.execute("UPDATE source_assets SET import_status='SUCCESS',error_code=NULL,error_detail=NULL WHERE id=?", (asset_id,))
        result["dji_version"] = inspection["version"]
        return result
