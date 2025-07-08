FROM python:3.6-slim

LABEL maintainer="azahir@pa.uc3m.es"

ENV DEBIAN_FRONTEND=noninteractive

SHELL [ "/bin/bash", "-c" ]

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3 \
    python3-pip \
    sudo \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy requirements file and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application code
COPY app/ .

# Set the entrypoint to execute main.py
ENTRYPOINT ["python3", "main.py"]