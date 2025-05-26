FROM python:3.10-slim

WORKDIR /app
COPY src/backend/dashboard/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY src/backend/dashboard/ .

EXPOSE 8501

CMD ["streamlit", "run", "streamlit.py", \
     "--server.port=8501", \
     "--server.address=0.0.0.0"]