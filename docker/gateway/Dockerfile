FROM python:3.12-slim AS builder
WORKDIR /build

#--- зависимости ---------------------------------------------------------
COPY src/backend/gateway/requirements.txt .
RUN pip install --user --no-cache-dir -r requirements.txt

# ---------- Runtime ----------
FROM python:3.12-slim AS runtime
ENV PYTHONUNBUFFERED=1
WORKDIR /app

#--- перенос установленных пакетов из builder-слоя -----------------------
COPY --from=builder /root/.local /usr/local

#--- копируем исходники сервиса ------------------------------------------
COPY src/backend/gateway/ .

#--- точка входа ----------------------------------------------------------
CMD ["uvicorn", "gateway_fastapi:app", "--host", "0.0.0.0", "--port", "8000"]