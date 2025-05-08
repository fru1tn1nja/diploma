"""Telemetry Collector Service
=============================
Подписывается на MQTT-топик(и) `telemetry/#`, парсит JSON и пишет в PostgreSQL.
"""
from __future__ import annotations

import asyncio
import json
import os
from typing import Any, Dict

from dotenv import load_dotenv
load_dotenv()                             # .env для локальной разработки

import asyncpg
from asyncio_mqtt import Client, MqttError

# ───── переменные окружения ────────────────────────────────────────────────
DB_DSN     = os.environ["DB_DSN"]
MQTT_HOST  = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT  = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "telemetry/#")   # wildcard

INSERT_SQL = (
    "INSERT INTO telemetry "
    "(device_id, ts, lat, lon, alt, roll, pitch, yaw, battery, vel) "
    "VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10)"
)

# ───── обработка одного сообщения ──────────────────────────────────────────
async def handle_message(pool: asyncpg.Pool, payload: bytes) -> None:
    try:
        data: Dict[str, Any] = json.loads(payload)
        args = (
            data["device_id"], data["ts"], data["lat"], data.get("lon"),
            data.get("alt"),  data.get("roll"),  data.get("pitch"),
            data.get("yaw"),  data.get("battery"), data.get("vel"),
        )
    except (KeyError, ValueError, json.JSONDecodeError) as exc:
        print(f"[collector] bad message: {exc}; payload={payload!r}")
        return

    async with pool.acquire() as conn:
        await conn.execute(INSERT_SQL, *args)

# ───── основной цикл ───────────────────────────────────────────────────────
async def main() -> None:
    print("[collector] starting…")
    pool = await asyncpg.create_pool(DB_DSN, min_size=1, max_size=5)

    async with Client(MQTT_HOST, port=MQTT_PORT) as client:
        await client.subscribe(MQTT_TOPIC)
        print(f"[collector] subscribed to {MQTT_TOPIC} on {MQTT_HOST}:{MQTT_PORT}")

        try:
            async with pool, client.unfiltered_messages() as messages:
                async for msg in messages:
                    asyncio.create_task(handle_message(pool, msg.payload))
        except MqttError as exc:
            print(f"[collector] MQTT error: {exc}")

if __name__ == "__main__":
    asyncio.run(main())
