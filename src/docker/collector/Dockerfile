# ---------- Builder ----------
    FROM python:3.12-slim AS builder
    WORKDIR /build
    
    COPY backend/collector/requirements.txt .
    RUN pip install --user --no-cache-dir -r requirements.txt
    
    # ---------- Runtime ----------
    FROM python:3.12-slim AS runtime
    ENV PYTHONUNBUFFERED=1
    WORKDIR /app
    
    # перенести установленные зависимости
    COPY --from=builder /root/.local /usr/local
    
    # копируем исходники сервиса
    COPY backend/collector/ .
    
    CMD ["python", "telemetry_collector_service.py"]
    