PRAGMA foreign_keys=ON;
PRAGMA journal_mode=WAL;

CREATE TABLE IF NOT EXISTS sites (
  id TEXT PRIMARY KEY, name TEXT NOT NULL, site_type TEXT NOT NULL DEFAULT 'CAGE_FARM',
  created_at TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'ACTIVE', notes TEXT
);
CREATE TABLE IF NOT EXISTS subcages (
  id TEXT PRIMARY KEY, site_id TEXT NOT NULL REFERENCES sites(id), label TEXT NOT NULL,
  row_index INTEGER, col_index INTEGER, notes TEXT, created_at TEXT NOT NULL,
  UNIQUE(site_id,label)
);
CREATE TABLE IF NOT EXISTS capture_sessions (
  id TEXT PRIMARY KEY, site_id TEXT REFERENCES sites(id), device TEXT,
  start_time TEXT, timezone_status TEXT, notes TEXT, created_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS import_jobs (
  id TEXT PRIMARY KEY, kind TEXT NOT NULL, state TEXT NOT NULL, created_at TEXT NOT NULL,
  finished_at TEXT, total INTEGER NOT NULL DEFAULT 0, succeeded INTEGER NOT NULL DEFAULT 0,
  failed INTEGER NOT NULL DEFAULT 0, result_json TEXT NOT NULL DEFAULT '{}'
);
CREATE TABLE IF NOT EXISTS source_assets (
  id TEXT PRIMARY KEY, sha256 TEXT NOT NULL UNIQUE, kind TEXT NOT NULL,
  original_name TEXT NOT NULL, original_path TEXT NOT NULL, source_role TEXT,
  byte_size INTEGER NOT NULL, mime_type TEXT, parser TEXT, parser_version TEXT,
  import_status TEXT NOT NULL, error_code TEXT, error_detail TEXT,
  metadata_json TEXT NOT NULL DEFAULT '{}', created_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS gps_samples (
  id TEXT PRIMARY KEY, source_asset_id TEXT NOT NULL REFERENCES source_assets(id),
  session_id TEXT REFERENCES capture_sessions(id), sample_index INTEGER,
  original_timestamp TEXT, timestamp_utc TEXT, time_quality TEXT,
  latitude REAL NOT NULL, longitude REAL NOT NULL, altitude_m REAL,
  altitude_reference TEXT, gps_valid INTEGER NOT NULL DEFAULT 1,
  gps_status_raw TEXT, coordinate_reference_status TEXT,
  satellites INTEGER, horizontal_accuracy_m REAL, speed_mps REAL,
  origin_type TEXT NOT NULL, quality_flags TEXT NOT NULL DEFAULT '[]',
  metadata_json TEXT NOT NULL DEFAULT '{}', UNIQUE(source_asset_id,sample_index)
);
CREATE TABLE IF NOT EXISTS photo_bindings (
  id TEXT PRIMARY KEY, photo_asset_id TEXT NOT NULL REFERENCES source_assets(id),
  subcage_id TEXT NOT NULL REFERENCES subcages(id), capture_marker TEXT,
  binding_confidence TEXT NOT NULL, reviewer TEXT NOT NULL, created_at TEXT NOT NULL,
  UNIQUE(photo_asset_id,subcage_id)
);
CREATE TABLE IF NOT EXISTS candidate_points (
  id TEXT PRIMARY KEY, subcage_id TEXT NOT NULL REFERENCES subcages(id),
  calculation_method TEXT NOT NULL, lat REAL NOT NULL, lon REAL NOT NULL,
  source_sample_ids TEXT NOT NULL, quality TEXT NOT NULL,
  status TEXT NOT NULL DEFAULT 'PENDING', created_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS approved_point_versions (
  id TEXT PRIMARY KEY, subcage_id TEXT NOT NULL REFERENCES subcages(id),
  candidate_point_id TEXT REFERENCES candidate_points(id), version INTEGER NOT NULL,
  lat REAL NOT NULL, lon REAL NOT NULL, evidence_json TEXT NOT NULL,
  reviewer TEXT NOT NULL, review_note TEXT, approved_at TEXT NOT NULL,
  active INTEGER NOT NULL DEFAULT 1, UNIQUE(subcage_id,version)
);
CREATE TABLE IF NOT EXISTS track_segments (
  id TEXT PRIMARY KEY, session_id TEXT NOT NULL REFERENCES capture_sessions(id),
  source_asset_id TEXT NOT NULL REFERENCES source_assets(id), segment_index INTEGER NOT NULL,
  start_time TEXT, end_time TEXT, sample_count INTEGER NOT NULL,
  distance_m REAL NOT NULL DEFAULT 0, break_reason TEXT, quality_flags TEXT NOT NULL DEFAULT '[]',
  UNIQUE(source_asset_id,segment_index)
);
CREATE TABLE IF NOT EXISTS track_segment_samples (
  segment_id TEXT NOT NULL REFERENCES track_segments(id), sample_id TEXT NOT NULL REFERENCES gps_samples(id),
  seq INTEGER NOT NULL, PRIMARY KEY(segment_id,seq)
);
CREATE TABLE IF NOT EXISTS route_previews (
  id TEXT PRIMARY KEY, site_id TEXT REFERENCES sites(id), name TEXT NOT NULL,
  home_lat REAL, home_lon REAL, include_return INTEGER NOT NULL DEFAULT 1,
  preview_only INTEGER NOT NULL DEFAULT 1, realflight_eligible INTEGER NOT NULL DEFAULT 0,
  total_distance_m REAL NOT NULL, warnings_json TEXT NOT NULL, created_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS route_preview_points (
  route_id TEXT NOT NULL REFERENCES route_previews(id), seq INTEGER NOT NULL,
  point_type TEXT NOT NULL, subcage_id TEXT, label TEXT NOT NULL,
  lat REAL NOT NULL, lon REAL NOT NULL, segment_distance_m REAL NOT NULL,
  approved_version_id TEXT, PRIMARY KEY(route_id,seq)
);
CREATE INDEX IF NOT EXISTS idx_samples_asset ON gps_samples(source_asset_id);
CREATE INDEX IF NOT EXISTS idx_candidates_subcage ON candidate_points(subcage_id);
CREATE INDEX IF NOT EXISTS idx_tracks_session ON track_segments(session_id);
