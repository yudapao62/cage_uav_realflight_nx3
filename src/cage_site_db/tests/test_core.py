#!/usr/bin/env python3
import csv
import json
import os
import shutil
import subprocess
import tempfile
import unittest

TEST_ROOT = tempfile.mkdtemp(prefix="cage_site_db_test_")
os.environ["CAGE_SITE_DB_DATA"] = TEST_ROOT

from PIL import Image
from cage_site_db import config
from cage_site_db.cli import backup_db
from cage_site_db.db import init_db, now, row, rows, transaction, uid
from cage_site_db.importers.csv_importer import import_csv
from cage_site_db.importers.dji_txt_adapter import inspect_txt
from cage_site_db.importers.exif_importer import import_photo
from cage_site_db.processing.tracks import segment_samples
from cage_site_db.service import create_route, route_csv, route_geojson, route_png


class CoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        init_db()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(TEST_ROOT, ignore_errors=True)

    def make_jpeg(self, name):
        path = os.path.join(TEST_ROOT, name)
        Image.new("RGB", (64, 48), "navy").save(path, "JPEG")
        return path

    def test_01_no_flight_control_imports(self):
        root = os.path.dirname(os.path.dirname(__file__))
        forbidden = ("import rospy", "from rospy", "import mavros", "from mavros", "import cage_msgs", "from cage_msgs")
        violations = []
        for base, _, files in os.walk(os.path.join(root, "cage_site_db")):
            for name in files:
                if not name.endswith(".py"): continue
                path = os.path.join(base, name)
                with open(path, encoding="utf-8") as handle:
                    text = handle.read().lower()
                if any(token in text for token in forbidden): violations.append(path)
        self.assertEqual([], violations)

    @unittest.skipUnless(shutil.which("exiftool"), "ExifTool required")
    def test_02_exif_no_gps_and_idempotency(self):
        path = self.make_jpeg("no-gps.jpg")
        first = import_photo(path, "no-gps.jpg", "CHAT_UPLOADED_COPY")
        second = import_photo(path, "same-bytes.jpg", "CHAT_UPLOADED_COPY")
        self.assertEqual("EXIF_NO_GPS", first["error_code"])
        self.assertTrue(second["idempotent"])
        with transaction() as conn:
            self.assertEqual(0, conn.execute("SELECT COUNT(*) FROM gps_samples WHERE source_asset_id=?", (first["id"],)).fetchone()[0])

    @unittest.skipUnless(shutil.which("exiftool"), "ExifTool required")
    def test_03_exif_gps_real_metadata(self):
        path = self.make_jpeg("with-gps.jpg")
        subprocess.run(["exiftool", "-overwrite_original", "-GPSLatitude=27.5825628333", "-GPSLatitudeRef=N",
                        "-GPSLongitude=120.5662079722", "-GPSLongitudeRef=E",
                        "-DateTimeOriginal=2026:10:27 13:57:43", path], check=True, stdout=subprocess.DEVNULL)
        result = import_photo(path, "with-gps.jpg", "TEST_FIXTURE")
        self.assertEqual("SUCCESS", result["import_status"])
        self.assertAlmostEqual(27.5825628333, result["latitude"], places=7)
        self.assertIn("EXIF_TIMEZONE_UNVERIFIED", result["quality_flags"])
        self.assertIn("PHOTO_CENTER_PROXY_UNVERIFIED", result["quality_flags"])

    def test_04_csv_normalization_and_segmentation(self):
        path = os.path.join(TEST_ROOT, "normalized.csv")
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle); writer.writerow(["sample_time_utc","latitude","longitude","gps_valid","source_sample_id"])
            writer.writerow(["2026-10-27T04:00:00Z",30.0001,120.0001,"true","a"])
            writer.writerow(["2026-10-27T04:00:00.2Z",30.000101,120.000101,"true","b"])
            writer.writerow(["2026-10-27T04:00:12.7Z",30.0005,120.0005,"true","c"])
        result = import_csv(path, "normalized.csv")
        self.assertEqual(3, result["valid_rows"])
        self.assertEqual(2, result["segments"])

    def test_05_raw_txt_detection(self):
        fake = os.path.join(TEST_ROOT, "DJIFlightRecord_fake.txt")
        with open(fake, "wb") as handle:
            handle.write(b"sample_time_utc,latitude,longitude\n")
        self.assertEqual("DJI_TXT_INVALID_FORMAT", inspect_txt(fake)["error_code"])
        encrypted = os.path.join(TEST_ROOT, "DJIFlightRecord_v13.txt")
        payload = bytearray(512); payload[10] = 13
        with open(encrypted, "wb") as handle:
            handle.write(payload)
        result = inspect_txt(encrypted)
        self.assertEqual(13, result["version"])
        self.assertIn(result.get("error_code"), ("DJI_TXT_DECRYPTION_KEY_REQUIRED", "DJI_TXT_PARSER_NOT_INSTALLED"))

    def test_06_track_reverse_duplicate_and_jump(self):
        samples = [
            {"id":"a","sample_index":0,"timestamp_utc":"2026-10-27T04:00:00+00:00","latitude":30.0,"longitude":120.0,"gps_valid":True},
            {"id":"b","sample_index":1,"timestamp_utc":"2026-10-27T04:00:00+00:00","latitude":30.0,"longitude":120.0,"gps_valid":True},
            {"id":"c","sample_index":2,"timestamp_utc":"2026-10-27T04:00:01+00:00","latitude":31.0,"longitude":121.0,"gps_valid":True},
        ]
        segments, diagnostics = segment_samples(samples)
        self.assertGreaterEqual(len(segments), 2)
        reasons = {item["reason"] for item in diagnostics}
        self.assertIn("DUPLICATE_SAMPLE", reasons)
        self.assertIn("DIAGNOSTIC_SPEED_JUMP", reasons)

    def test_07_review_and_route_exports_never_flight_eligible(self):
        with transaction() as conn:
            site=uid("site"); cage=uid("subcage"); candidate=uid("candidate"); approved=uid("approved")
            conn.execute("INSERT INTO sites VALUES (?,?,?,?,?,?)",(site,"route-test","TEST",now(),"ACTIVE",None))
            conn.execute("INSERT INTO subcages VALUES (?,?,?,?,?,?,?)",(cage,site,"C-01",None,None,None,now()))
            conn.execute("INSERT INTO candidate_points VALUES (?,?,?,?,?,?,?,?,?)",(candidate,cage,"TEST_EVIDENCE",27.58,120.56,"[]","TEST_REAL_STRUCTURE","APPROVED",now()))
            conn.execute("INSERT INTO approved_point_versions VALUES (?,?,?,?,?,?,?,?,?,?,?)",(approved,cage,candidate,1,27.58,120.56,"{}","tester","unit test",now(),1))
        route = create_route({"site_id":site,"subcage_ids":[cage],"home":{"lat":27.579,"lon":120.559},"include_return":True})
        self.assertTrue(route["preview_only"])
        self.assertFalse(route["realflight_eligible"])
        self.assertIn(b"realflight_eligible=false", route_csv(route))
        self.assertFalse(json.loads(route_geojson(route))["features"][0]["properties"]["realflight_eligible"])
        self.assertTrue(route_png(route).startswith(b"\x89PNG"))

    def test_08_sqlite_backup_integrity(self):
        result = backup_db(os.path.join(TEST_ROOT, "backup.sqlite"))
        self.assertEqual("ok", result["integrity_check"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
