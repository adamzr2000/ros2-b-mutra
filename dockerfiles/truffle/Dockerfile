# Use the specific version of the Ubuntu 20.04 base image from Docker Hub
FROM ubuntu:20.04

LABEL maintainer="azahir@pa.uc3m.es"

# Set environment variables to avoid user interaction during the installation
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    python3 \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies from requirements.txt using pip3
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Download Node.js setup script for version 18.x from NodeSource
RUN curl -sL https://deb.nodesource.com/setup_18.x -o /tmp/nodesource_setup.sh

# Run the downloaded setup script to prepare for Node.js installation
RUN bash /tmp/nodesource_setup.sh

# Install Node.js
RUN apt install nodejs

# Install Truffle globally using npm
RUN npm install -g truffle

# Set the working directory inside the container
WORKDIR /smart-contracts

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"