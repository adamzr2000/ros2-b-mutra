FROM python:3.11-slim

# Set bash as default shell, and avoid prompts during package installation
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

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

WORKDIR /src

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application code
COPY app/ ./app

ENTRYPOINT ["python3", "-m", "uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]