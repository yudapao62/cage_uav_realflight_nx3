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
MAX_COORDINATE_CSV_BYTES = 2 * 1024 * 1024
MAX_BATCH_FILES = 100


def ensure_dirs():
    paths = [
        "incoming/photos", "incoming/coordinate_csv",
        "originals", "normalized", "normalized/thumbnails", "db/backups",
        "reports", "previews", "test_fixtures", "verified_examples",
    ]
    for path in paths:
        os.makedirs(os.path.join(ROOT, path), exist_ok=True)
