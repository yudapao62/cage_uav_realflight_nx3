import contextlib
import datetime as dt
import json
import os
import sqlite3
import uuid

from . import config


def now():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def uid(prefix):
    return "%s_%s" % (prefix, uuid.uuid4().hex)


def connect():
    config.ensure_dirs()
    conn = sqlite3.connect(config.DB_PATH, timeout=30)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


def init_db():
    config.ensure_dirs()
    existed = os.path.isfile(config.DB_PATH) and os.path.getsize(config.DB_PATH) > 0
    with connect() as conn:
        existed = bool(existed and conn.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='subcages'").fetchone())
        if existed:
            from .migrations import backup_before_migration, migrate
            pending, _ = backup_before_migration(conn)
            migrate(conn, pending)
        else:
            pending = None
        with open(config.SCHEMA_PATH, encoding="utf-8") as handle:
            conn.executescript(handle.read())
        if not existed:
            from .migrations import migrate
            migrate(conn, pending)


@contextlib.contextmanager
def transaction():
    conn = connect()
    try:
        yield conn
        conn.commit()
    except Exception:
        conn.rollback()
        raise
    finally:
        conn.close()


def rows(cur):
    return [dict(row) for row in cur.fetchall()]


def row(cur):
    value = cur.fetchone()
    return dict(value) if value else None


def dump(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def load(value, default=None):
    try:
        return json.loads(value)
    except Exception:
        return default
