import argparse
import json
import os
import sqlite3

from . import SAFETY_MODE, __version__, config
from .db import init_db, now, transaction, uid
from .importers.csv_importer import import_csv
from .importers.dji_txt_adapter import import_txt, inspect_txt
from .importers.exif_importer import import_photo


def output(value):
    print(json.dumps(value, ensure_ascii=False, indent=2, default=str))


def backup_db(target=None):
    init_db()
    target = target or os.path.join(config.ROOT, "db", "backups", "cage_site_%s.sqlite" % now().replace(":", "-").replace("+", "_"))
    os.makedirs(os.path.dirname(target), exist_ok=True)
    source = sqlite3.connect(config.DB_PATH)
    dest = sqlite3.connect(target)
    with dest:
        source.backup(dest)
    source.close(); dest.close()
    check = sqlite3.connect(target)
    integrity = check.execute("PRAGMA integrity_check").fetchone()[0]
    check.close()
    return {"ok": True, "path": target, "integrity_check": integrity}


def demo_data():
    init_db(); site_id = uid("site")
    with transaction() as conn:
        conn.execute("INSERT INTO sites VALUES (?,?,?,?,?,?)", (site_id, "SIMULATED_DEMO_SITE", "DEMO", now(), "SIMULATED", "Never use for real flight"))
        for label, lat, lon in (("SIM-01", 30.0001, 120.0001), ("SIM-02", 30.0002, 120.0003), ("SIM-03", 30.0004, 120.0005)):
            cage_id = uid("subcage"); candidate_id = uid("candidate")
            conn.execute("INSERT INTO subcages VALUES (?,?,?,?,?,?,?)", (cage_id, site_id, label, None, None, "SIMULATED", now()))
            conn.execute("INSERT INTO candidate_points VALUES (?,?,?,?,?,?,?,?,?)", (candidate_id, cage_id, "SIMULATED_FIXTURE", lat, lon, "[]", "SIMULATED_UNAPPROVABLE", "PENDING", now()))
    return {"ok": True, "site_id": site_id, "warning": "SIMULATED_UNAPPROVABLE", "mode": SAFETY_MODE}


def main(argv=None):
    parser = argparse.ArgumentParser(description="CAGE site DB: DATA_ONLY/PREVIEW_ONLY")
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("init-db")
    photo = commands.add_parser("import-photo"); photo.add_argument("--file", required=True); photo.add_argument("--source-role", default="CLI_UPLOAD")
    inspect = commands.add_parser("inspect-txt"); inspect.add_argument("--file", required=True)
    txt = commands.add_parser("import-txt"); txt.add_argument("--file", required=True); txt.add_argument("--site-id")
    csvp = commands.add_parser("import-csv"); csvp.add_argument("--file", required=True); csvp.add_argument("--site-id")
    backup = commands.add_parser("backup"); backup.add_argument("--output")
    commands.add_parser("demo-data")
    args = parser.parse_args(argv); init_db()
    if args.command == "init-db": result = {"ok": True, "db": config.DB_PATH, "mode": SAFETY_MODE}
    elif args.command == "import-photo": result = import_photo(args.file, os.path.basename(args.file), args.source_role)
    elif args.command == "inspect-txt": result = inspect_txt(args.file)
    elif args.command == "import-txt": result = import_txt(args.file, os.path.basename(args.file), args.site_id)
    elif args.command == "import-csv": result = import_csv(args.file, os.path.basename(args.file), args.site_id)
    elif args.command == "backup": result = backup_db(args.output)
    else: result = demo_data()
    output(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
