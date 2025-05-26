import os
import streamlit as st
import requests
import pandas as pd
from datetime import datetime

st.set_page_config(layout="wide", page_title="⛵ Vessel Dashboard")

# ── Configuration ─────────────────────────────────────────────────────────────
API_BASE  = os.getenv("TELEMETRY_API", "gateway:8000")
# Let user pick which device (boat) to inspect
DEVICE_ID = 1
# How many history points to load
HISTORY_N = st.sidebar.slider("History points", min_value=100, max_value=2000, value=500, step=100)

# ── Data loaders ───────────────────────────────────────────────────────────────
@st.cache_data(ttl=5)
def get_latest(device_id: int) -> dict:
    resp = requests.get(f"http://{API_BASE}/api/telemetry/latest", params={"device_id": device_id}, timeout=3)
    resp.raise_for_status()
    return resp.json()

@st.cache_data(ttl=5)
def get_history(device_id: int, limit: int) -> pd.DataFrame:
    resp = requests.get(
        f"http://{API_BASE}/api/telemetry/history",
        params={"device_id": device_id, "limit": limit},
        timeout=5
    )
    resp.raise_for_status()
    df = pd.DataFrame(resp.json())
    df["ts"] = pd.to_datetime(df["ts"], unit="ms")
    return df.set_index("ts").sort_index()

# ── Fetch data ─────────────────────────────────────────────────────────────────
try:
    latest = get_latest(DEVICE_ID)
    hist = get_history(DEVICE_ID, HISTORY_N)
    error = None
except Exception as e:
    latest, hist = None, None
    error = e

# ── Header & Metrics ──────────────────────────────────────────────────────────
st.title(f"⛵ Vessel Telemetry")
if error:
    st.error(f"❌ Could not load data: {error}")
    st.stop()

col1, col2, col3, col4 = st.columns(4)
col1.metric("🔋 Battery (%)", f"{latest['battery']}%", delta=f"{latest['battery']-hist['battery'].iloc[-1]:+.0f}")
col2.metric("💨 Velocity (m/s)", f"{latest['vel']:.2f}", delta=f"{latest['vel']-hist['battery'].iloc[-1]:+.2f}")
col3.metric("📍 Latitude", f"{latest['lat']:.6f}")
col4.metric("📍 Longitude", f"{latest['lon']:.6f}")

# ── Main charts ────────────────────────────────────────────────────────────────
st.subheader("Battery History")
chart_type = st.selectbox("Chart type", ["Line", "Bar"], key="chart_type")

if chart_type == "Line":
    st.line_chart(hist["battery"], use_container_width=True)
else:
    st.bar_chart(hist["battery"], use_container_width=True)



# ── Map ────────────────────────────────────────────────────────────────────────
# st.subheader("Current Position")
# df_map = pd.DataFrame([{"lat": latest["lat"], "lon": latest["lon"]}])
# st.map(df_map, zoom=10)

# ── JSON dump ──────────────────────────────────────────────────────────────────
with st.expander("View raw JSON"):
    st.json(latest)

# ── Footer / navigation ─────────────────────────────────────────────────────────
primary = st.get_option("theme.primaryColor") or "#f63366"
st.markdown(f"""
    <style>
    .back-btn {{
      display: inline-block;
      margin: 2em 0;
      padding: 0.6em 1.2em;
      background-color: {primary};
      color: white !important;
      border-radius: 0.25rem;
      text-decoration: none !important;
      font-weight: 500;
    }}
    .back-btn:hover {{
      filter: brightness(0.9);
      text-decoration: none !important;
    }}
    </style>
    <a href="http://localhost:3000" class="back-btn">← Back to Dashboard</a>
""", unsafe_allow_html=True)
