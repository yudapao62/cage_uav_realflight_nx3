import os

ROOT = os.environ.get("CAGE_SITE_DB_DATA", "/home/nx3/cage_uav_ws/data/dji_gps")
DB_PATH = os.path.join(ROOT, "db", "cage_site.db")
SCHEMA_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "schema.sql")
ORIGINALS = os.path.join(ROOT, "originals")
NORMALIZED = os.path.join(ROOT, "normalized")
REPORTS = os.path.join(ROOT, "reports")
PREVIEWS = os.path.join(ROOT, "previews")
THUMBNAILS = os.path.join(ROOT, "normalized", "thumbnails")
MAX_PHOTO_BYTES = 40 * 1024 * 1024
MAX_TXT_BYTES = 300 * 1024 * 1024
MAX_CSV_BYTES = 200 * 1024 * 1024
MAX_BATCH_FILES = 100
PARSER_BIN = os.environ.get("DJI_LOG_PARSER_BIN", "/home/nx3/cage_uav_ws/tools/dji-log-v0.5.7")
PARSER_NAME = "lvauvillier/DJI-Log-Parser"
PARSER_VERSION = "v0.5.7+e2e0775670a8391b4f7ecc40fca4cb01ea4a90fa"


def ensure_dirs():
    paths = [
        "incoming/photos", "incoming/flight_txt", "incoming/converted",
        "originals", "normalized", "normalized/thumbnails", "db/backups",
        "reports", "previews", "test_fixtures", "verified_examples",
    ]
    for path in paths:
        os.makedirs(os.path.join(ROOT, path), exist_ok=True)
