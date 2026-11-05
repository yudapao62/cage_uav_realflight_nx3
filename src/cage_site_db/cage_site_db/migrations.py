"""Small, transactional SQLite migrations for field databases."""
import os
import sqlite3

from . import config
from .db import now


MIGRATIONS = (
    (1, "subcage_status", (
        "ALTER TABLE subcages ADD COLUMN status TEXT NOT NULL DEFAULT 'ACTIVE'",
        "CREATE INDEX IF NOT EXISTS idx_subcages_site_status ON subcages(site_id,status,label)",
    )),
    (2, "mission_snapshots", (
        """CREATE TABLE IF NOT EXISTS mission_snapshots (
          id TEXT PRIMARY KEY, mission_id TEXT NOT NULL UNIQUE,
          input_profile TEXT NOT NULL, site_id TEXT REFERENCES sites(id),
          home_lat REAL NOT NULL, home_lon REAL NOT NULL, home_alt REAL,
          takeoff_relative_alt REAL NOT NULL, transit_relative_alt REAL NOT NULL,
          feed_relative_alt REAL NOT NULL, feeding_dwell_sec REAL NOT NULL,
          waypoint_hold_sec REAL NOT NULL, speed REAL NOT NULL,
          acceptance_radius REAL NOT NULL, preview_hash TEXT NOT NULL,
          state TEXT NOT NULL, details_json TEXT NOT NULL DEFAULT '{}',
          created_at TEXT NOT NULL, updated_at TEXT NOT NULL
        )""",
        """CREATE TABLE IF NOT EXISTS mission_snapshot_targets (
          snapshot_id TEXT NOT NULL REFERENCES mission_snapshots(id), seq INTEGER NOT NULL,
          subcage_id TEXT REFERENCES subcages(id), approved_version_id TEXT,
          label TEXT NOT NULL, lat REAL NOT NULL, lon REAL NOT NULL,
          PRIMARY KEY(snapshot_id,seq)
        )""",
        "CREATE INDEX IF NOT EXISTS idx_mission_snapshots_site ON mission_snapshots(site_id,created_at)",
    )),
)


def _columns(conn, table):
    return {item[1] for item in conn.execute("PRAGMA table_info(%s)" % table)}


def pending_versions(conn):
    conn.execute("""CREATE TABLE IF NOT EXISTS schema_migrations (
        version INTEGER PRIMARY KEY, name TEXT NOT NULL, applied_at TEXT NOT NULL)""")
    applied = {item[0] for item in conn.execute("SELECT version FROM schema_migrations")}
    pending = []
    for version, name, statements in MIGRATIONS:
        if version in applied:
            continue
        if version == 1 and "status" in _columns(conn, "subcages"):
            statements = statements[1:]
        pending.append((version, name, statements))
    return pending


def backup_before_migration(conn):
    pending = pending_versions(conn)
    if not pending or not os.path.isfile(config.DB_PATH):
        return pending, None
    backup_dir = os.path.join(config.ROOT, "db", "backups")
    os.makedirs(backup_dir, exist_ok=True)
    suffix = now().replace(":", "-").replace("+", "_")
    backup_path = os.path.join(backup_dir, "pre_migration_%s.sqlite" % suffix)
    target = sqlite3.connect(backup_path)
    try:
        conn.backup(target)
    finally:
        target.close()
    return pending, backup_path


def migrate(conn, pending=None):
    pending = pending if pending is not None else pending_versions(conn)
    for version, name, statements in pending:
        for statement in statements:
            conn.execute(statement)
        conn.execute("INSERT INTO schema_migrations(version,name,applied_at) VALUES (?,?,?)",
                     (version, name, now()))
    return [version for version, _, _ in pending]
