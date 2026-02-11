# ---------------------------------------------------------
# Stage 1: Builder
# ---------------------------------------------------------
FROM golang:1.23-bookworm AS builder

WORKDIR /src

# Copy the application source code
COPY app/ ./app/

# Initialize Go Module (if not already done locally)
# We move into the folder where main.go resides
WORKDIR /src/app

# 1. Init module 'app' if go.mod doesn't exist
# 2. Pin go-ethereum to a compatible version (v1.14.12 is stable for Go 1.23)
# 3. Tidy dependencies
# 4. Build the binary named 'sidecar'
RUN [ ! -f go.mod ] && go mod init app || true \
    && go get github.com/ethereum/go-ethereum@v1.14.12 \
    && go mod tidy \
    && CGO_ENABLED=0 go build -o /bin/attestation-sidecar .

# ---------------------------------------------------------
# Stage 2: Runtime
# ---------------------------------------------------------
FROM debian:bookworm-slim

# Set bash as default shell
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies
# - binutils: REQUIRED for 'readelf' (used by app/internal/compute/hash.go)
# - ca-certificates: REQUIRED for HTTPS blockchain connections
# - redis-tools: Optional, useful for debugging
RUN apt-get update && apt-get install -y --no-install-recommends \
    binutils \
    ca-certificates \
    redis-tools \
    procps \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy the compiled binary from the builder stage
COPY --from=builder /bin/attestation-sidecar .

# Expose the port configured in main.go (Default 8000)
# Note: Your Python version used 8080, but your Go main.go uses 8000.
EXPOSE 8000

# Run the binary
ENTRYPOINT ["./attestation-sidecar"]