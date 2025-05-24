BEGIN;

-- ────────────────────────────── TELEMETRY ────────────────────────────────
CREATE TABLE IF NOT EXISTS telemetry (
    id        bigserial PRIMARY KEY,
    device_id integer          NOT NULL,
    ts        bigint           NOT NULL,  -- epoch‑ms
    lat       double precision NOT NULL,
    lon       double precision,
    alt       double precision,
    roll      double precision,
    pitch     double precision,
    yaw       double precision,
    battery   integer,
    vel       double precision
);

CREATE INDEX IF NOT EXISTS telemetry_device_idx ON telemetry (device_id);
CREATE INDEX IF NOT EXISTS telemetry_ts_idx     ON telemetry (ts DESC);

-- ───────────────────────────── CURRENT MODE ──────────────────────────────
CREATE TABLE IF NOT EXISTS current_mode (
    id   boolean  PRIMARY KEY DEFAULT TRUE,
    mode integer  NOT NULL    DEFAULT 1             -- 1 = Manual
);

-- Заполняем/исправляем существующую строку
INSERT INTO current_mode(id,mode) VALUES (TRUE,1)
ON CONFLICT (id)            -- если строка уже была
DO UPDATE SET mode = EXCLUDED.mode;