"""FastAPI Gateway
=================
REST- и WebSocket-шлюз между фронтендом и сервисами.

*  GET  /api/telemetry/latest?device_id=…   — последняя запись телеметрии
*  WS   /ws/telemetry/{device_id}           — живой поток MQTT → WS
"""
from __future__ import annotations
from typing import List
import os
from typing import Any, Dict
import asyncio 
from dotenv import load_dotenv
load_dotenv()                       

import asyncpg
from asyncio_mqtt import Client, MqttError
from fastapi import Depends, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

# ───── переменные окружения ────────────────────────────────────────────────
DB_DSN     = os.environ["DB_DSN"]                  # postgresql://…/diploma
MQTT_HOST  = os.getenv("MQTT_HOST", "localhost")   # в Docker → emqx
MQTT_PORT  = int(os.getenv("MQTT_PORT", "1883"))
API_HOST   = os.getenv("API_HOST", "0.0.0.0")
API_PORT   = int(os.getenv("API_PORT", "8000"))
CORS_ORIGS = os.getenv("API_CORS_ORIGINS", "*").split(",")

# ───── FastAPI / CORS ──────────────────────────────────────────────────────
app = FastAPI(title="Shared-Control Gateway", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGS,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ───── PostgreSQL pool ─────────────────────────────────────────────────────
async def get_pool() -> asyncpg.Pool:
    if not hasattr(app.state, "pool"):
        app.state.pool = await asyncpg.create_pool(DB_DSN, min_size=1, max_size=10)  # type: ignore[attr-defined]
    return app.state.pool  # type: ignore[attr-defined]

@app.on_event("shutdown")
async def close_pool():
    if hasattr(app.state, "pool"):
        await app.state.pool.close()  # type: ignore[attr-defined]



# ───── REST: последняя телеметрия ──────────────────────────────────────────

@app.websocket("/ws/mode")
async def ws_mode(ws: WebSocket):
    await ws.accept()
    conn: asyncpg.Connection | None = None
    try:
        conn = await asyncpg.connect(dsn=DB_DSN)
        # слушаем канал pg_notify (можно LISTEN) или опрашиваем таблицу switches
        prev = None
        while True:
            row = await conn.fetchrow("SELECT mode FROM current_mode LIMIT 1")
            mode = row["mode"] if row else "1"
            if mode != prev:
                await ws.send_text(mode)
                prev = mode
            await asyncio.sleep(0.5)
    finally:
        if conn: await conn.close()
        await ws.close()

        
@app.get("/api/telemetry/latest")
async def telemetry_latest(device_id: int,
                           pool: asyncpg.Pool = Depends(get_pool)) -> Dict[str, Any]:
    row = await pool.fetchrow(
        "SELECT * FROM telemetry WHERE device_id=$1 ORDER BY ts DESC LIMIT 1",
        device_id,
    )
    if row is None:
        raise HTTPException(status_code=404, detail="device not found or no data")
    return dict(row)

# ───── WebSocket: живой поток ──────────────────────────────────────────────
@app.websocket("/ws/telemetry/{device_id}")
async def telemetry_ws(ws: WebSocket, device_id: int):
    await ws.accept()
    topic = f"telemetry/{device_id}"
    try:
        async with Client(MQTT_HOST, port=MQTT_PORT) as client:
            await client.subscribe(topic)
            async with client.unfiltered_messages() as messages:
                async for msg in messages:
                    await ws.send_text(msg.payload.decode())
    except (MqttError, WebSocketDisconnect):
        pass
    finally:
        await ws.close()

@app.websocket("/ws/mission/{device_id}")
async def mission_ws(ws: WebSocket, device_id: int):
    await ws.accept()
    topic = f"mission/waypoints/{device_id}"
    try:
        async with Client(MQTT_HOST, port=MQTT_PORT) as client:
            await client.subscribe(topic)
            async with client.unfiltered_messages() as messages:
                async for msg in messages:
                    await ws.send_text(msg.payload.decode())
    except (MqttError, WebSocketDisconnect):
        pass
    finally:
        await ws.close()

@app.websocket("/ws/obstacles/{device_id}")
async def ws_obstacles(ws: WebSocket, device_id: int):
    await ws.accept()
    topic = f"obstacles/{device_id}"
    try:
        async with Client(MQTT_HOST, port=MQTT_PORT) as client:
            await client.subscribe(topic)
            async with client.unfiltered_messages() as msgs:
                async for msg in msgs:
                    await ws.send_text(msg.payload.decode())
    except (MqttError, WebSocketDisconnect):
        pass
    finally:
        await ws.close()

# ───── локальный запуск ────────────────────────────────────────────────────
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("gateway_fastapi:app", host=API_HOST, port=API_PORT, reload=True)
