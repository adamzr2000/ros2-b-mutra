.PHONY: all clean attestation-sidecar docker-stats-collector gazebo-vnc turtlebot3 hardhat secaas secaas-wrapper

# --- Variables ---
IMG_ATTESTATION_SIDECAR    := attestation-sidecar:latest
IMG_DOCKER_STATS_COLLECTOR := docker-stats-collector:latest
IMG_GAZEBO                 := gazebo-vnc:latest
IMG_TURTLEBOT              := turtlebot3-gazebo:latest
IMG_HARDHAT                := hardhat:latest
IMG_SECAAS                 := secaas:latest
IMG_SECAAS_WRAPPER         := secaas-wrapper:latest

# --- Main Command ---

# Default target: Builds all in parallel and prints summary on success
all: 
	@echo ">> 🚀 Starting parallel build for all images..."
	@$(MAKE) -j attestation-sidecar docker-stats-collector gazebo-vnc turtlebot3 hardhat secaas secaas-wrapper
	@echo ""
	@echo "========================================================"
	@echo "   ✅  ALL IMAGES BUILT SUCCESSFULLY"
	@echo "========================================================"

# Remove the specific images built by this file
clean:
	@echo ">> 🧹 Removing project images..."
	-docker rmi $(IMG_ATTESTATION_SIDECAR) $(IMG_DOCKER_STATS_COLLECTOR) $(IMG_GAZEBO) $(IMG_TURTLEBOT) $(IMG_HARDHAT) $(IMG_SECAAS) $(IMG_SECAAS_WRAPPER)

# --- Individual Build Rules ---

attestation-sidecar:
	@echo ">> 📦 Building [$(IMG_ATTESTATION_SIDECAR)]..."
	@docker build -t $(IMG_ATTESTATION_SIDECAR) ./attestation-sidecar

docker-stats-collector:
	@echo ">> 📦 Building [$(IMG_DOCKER_STATS_COLLECTOR)]..."
	@docker build -t $(IMG_DOCKER_STATS_COLLECTOR) ./docker-stats-collector

gazebo-vnc:
	@echo ">> 📦 Building [$(IMG_GAZEBO)]..."
	@docker build -t $(IMG_GAZEBO) ./gazebo-vnc

turtlebot3:
	@echo ">> 📦 Building [$(IMG_TURTLEBOT)] (from ./turtlebot3)..."
	@docker build -t $(IMG_TURTLEBOT) ./turtlebot3

hardhat:
	@echo ">> 📦 Building [$(IMG_HARDHAT)]..."
	@docker build -t $(IMG_HARDHAT) ./hardhat

secaas:
	@echo ">> 📦 Building [$(IMG_SECAAS)]..."
	@docker build -t $(IMG_SECAAS) ./secaas

secaas-wrapper:
	@echo ">> 📦 Building [$(IMG_SECAAS_WRAPPER)]..."
	@docker build -t $(IMG_SECAAS_WRAPPER) ./secaas-wrapper