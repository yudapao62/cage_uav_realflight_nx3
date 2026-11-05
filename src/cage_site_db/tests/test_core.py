#!/usr/bin/env python3
import os
import shutil
import sqlite3
import subprocess
import tempfile
import unittest

TEST_ROOT = tempfile.mkdtemp(prefix="cage_site_db_test_")
os.environ["CAGE_SITE_DB_DATA"] = TEST_ROOT

from PIL import Image
from cage_site_db import config
from cage_site_db.cli import backup_db
from cage_site_db.db import init_db, now, row, transaction, uid
from cage_site_db.importers.coordinate_csv import import_coordinate_csv
from cage_site_db.importers.exif_importer import import_photo
from cage_site_db.migrations import migrate, pending_versions


class CoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): init_db()

    @classmethod
    def tearDownClass(cls): shutil.rmtree(TEST_ROOT, ignore_errors=True)

    def make_site(self, suffix):
        site = uid("site")
        with transaction() as conn:
            conn.execute("INSERT INTO sites VALUES (?,?,?,?,?,?)",
                         (site, "large-"+suffix, "CAGE_FARM", now(), "ACTIVE", None))
        return site

    def make_subcage(self, site, label):
        cage = uid("subcage")
        with transaction() as conn:
            conn.execute("""INSERT INTO subcages
                (id,site_id,label,row_index,col_index,notes,created_at,status)
                VALUES (?,?,?,?,?,?,?,'ACTIVE')""",
                (cage, site, label, None, None, None, now()))
        return cage

    def make_jpeg(self, name):
        path = os.path.join(TEST_ROOT, name)
        Image.new("RGB", (64, 48), "navy").save(path, "JPEG")
        return path

    def test_no_flight_control_imports(self):
        root = os.path.dirname(os.path.dirname(__file__))
        forbidden = ("import rospy", "from rospy", "import mavros", "from mavros",
                     "import cage_msgs", "from cage_msgs")
        violations = []
        for base, _, files in os.walk(os.path.join(root, "cage_site_db")):
            for name in files:
                if not name.endswith(".py"): continue
                with open(os.path.join(base, name), encoding="utf-8") as handle:
                    text = handle.read().lower()
                if any(token in text for token in forbidden): violations.append(name)
        self.assertEqual([], violations)

    @unittest.skipUnless(shutil.which("exiftool"), "ExifTool required")
    def test_photo_xy_and_idempotency(self):
        path = self.make_jpeg("with-gps.jpg")
        subprocess.run(["exiftool", "-overwrite_original", "-GPSLatitude=27.5825628333",
                        "-GPSLatitudeRef=N", "-GPSLongitude=120.5662079722",
                        "-GPSLongitudeRef=E", "-GPSAltitude=888", path], check=True,
                       stdout=subprocess.DEVNULL)
        first = import_photo(path, "with-gps.jpg", "TEST")
        second = import_photo(path, "same.jpg", "TEST")
        self.assertEqual("SUCCESS", first["import_status"])
        self.assertAlmostEqual(27.5825628333, first["latitude"], places=7)
        self.assertTrue(second["idempotent"])

    def test_coordinate_csv_200_rows_pending_and_line_errors(self):
        site = self.make_site("csv")
        for index in range(1, 201): self.make_subcage(site, "A-%03d" % index)
        lines = ["small_cage_label,latitude,longitude,row_index,col_index,note"]
        lines += ["A-%03d,27.%08d,120.%08d,1,%d,test" %
                  (index, 58000000+index, 56000000+index, index)
                  for index in range(1, 201)]
        lines.append("MISSING,999,999,,,")
        result = import_coordinate_csv("\n".join(lines).encode("utf-8"), site)
        self.assertEqual(200, result["succeeded"])
        self.assertEqual(1, result["failed"])
        with transaction() as conn:
            self.assertEqual(200, conn.execute("SELECT COUNT(*) FROM candidate_points WHERE calculation_method='MANUAL_WGS84_CSV' AND status='PENDING'").fetchone()[0])

    def test_manual_candidate_version_history(self):
        site = self.make_site("manual"); cage = self.make_subcage(site, "M-001")
        with transaction() as conn:
            for version in (1, 2):
                candidate = uid("candidate")
                conn.execute("INSERT INTO candidate_points VALUES (?,?,?,?,?,?,?,?,?)",
                             (candidate, cage, "MANUAL_WGS84", 27.58+version/10000.0,
                              120.56, "{}", "USER_REVIEW_REQUIRED", "APPROVED", now()))
                conn.execute("UPDATE approved_point_versions SET active=0 WHERE subcage_id=?", (cage,))
                conn.execute("INSERT INTO approved_point_versions VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                             (uid("approved"), cage, candidate, version, 27.58+version/10000.0,
                              120.56, "{}", "tester", "review", now(), 1))
            versions = conn.execute("SELECT version,active FROM approved_point_versions WHERE subcage_id=? ORDER BY version", (cage,)).fetchall()
        self.assertEqual([(1, 0), (2, 1)], [tuple(item) for item in versions])

    def test_legacy_schema_migrates_without_dropping_tracks(self):
        conn = sqlite3.connect(":memory:")
        conn.executescript("""CREATE TABLE subcages(id TEXT PRIMARY KEY,site_id TEXT,label TEXT,row_index INTEGER,col_index INTEGER,notes TEXT,created_at TEXT);
            CREATE TABLE track_segments(id TEXT PRIMARY KEY); INSERT INTO track_segments VALUES ('legacy');""")
        pending = pending_versions(conn); migrate(conn, pending)
        columns = {item[1] for item in conn.execute("PRAGMA table_info(subcages)")}
        self.assertIn("status", columns)
        self.assertEqual("legacy", conn.execute("SELECT id FROM track_segments").fetchone()[0])
        self.assertTrue(conn.execute("SELECT name FROM sqlite_master WHERE name='mission_snapshots'").fetchone())
        conn.close()

    def test_sqlite_backup_integrity(self):
        result = backup_db(os.path.join(TEST_ROOT, "backup.sqlite"))
        self.assertEqual("ok", result["integrity_check"])


if __name__ == "__main__": unittest.main(verbosity=2)
