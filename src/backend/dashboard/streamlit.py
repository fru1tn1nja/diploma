import os
import streamlit as st
import requests
import pandas as pd

st.title("🚤 Battery Charge Over Time")

# Read API base URL from env (set in Docker Compose)
API_BASE  = os.getenv("TELEMETRY_API", "gateway:8000")
DEVICE_ID = 1
HISTORY_N = st.sidebar.slider("Points to fetch",  100, 1000, 500, 50)

@st.cache_data(ttl=1)
def get_history(device_id: int, n: int) -> pd.DataFrame:
    r = requests.get(
        f"http://{API_BASE}/api/telemetry/history",
        params={"device_id": device_id, "limit": n},
        timeout=3
    )
    r.raise_for_status()
    data = r.json()
    df = pd.DataFrame(data)
    df["ts"] = pd.to_datetime(df["ts"], unit="ms")
    return df.set_index("ts")

try:
    df = get_history(DEVICE_ID, HISTORY_N)
    st.line_chart(df["battery"])
except Exception as e:
    st.error(f"Failed to fetch telemetry: {e}")

if st.button("← Back to Main App"):
    st.markdown("[Go back to Dashboard](/)")