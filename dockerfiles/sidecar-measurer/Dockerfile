FROM python:3.6-slim

# Installer les dépendances nécessaires
RUN apt-get update && apt-get install -y --no-install-recommends \
    redis-tools \
    procps \
    jq \
    docker.io \
    binutils \
    && rm -rf /var/lib/apt/lists/*

COPY app /app

# Copier les dépendances Python
COPY requirements.txt /app

# Installer les packages Python
RUN pip install --no-cache-dir -r /app/requirements.txt

# Définir le répertoire de travail
WORKDIR /app