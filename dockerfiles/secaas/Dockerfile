FROM python:3.6-slim

LABEL maintainer="azahir@pa.uc3m.es"

ENV DEBIAN_FRONTEND=noninteractive

SHELL [ "/bin/bash", "-c" ]

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    redis-tools \
    procps \
    jq \
    docker.io \
    binutils \
    sudo \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy requirements file and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application code
COPY app/ .

ENTRYPOINT ["python3", "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8080"]
