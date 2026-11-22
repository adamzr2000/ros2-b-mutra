.PHONY: all clean blockchain quorum-explorer attestation-sidecar docker-stats-collector gazebo-server gazebo-gui robot-simulation hardhat secaas orchestrator

# --- Variables ---
IMG_ATTESTATION_SIDECAR    := attestation-sidecar:latest
IMG_DOCKER_STATS_COLLECTOR := docker-stats-collector:latest
IMG_GAZEBO_SERVER          := gazebo-server:latest
IMG_GAZEBO_GUI             := gazebo-gui:latest
IMG_ROBOT_SIMULATION       := robot-simulation:latest
IMG_HARDHAT                := hardhat:latest
IMG_SECAAS                 := secaas:latest
IMG_ORCHESTRATOR           := orchestrator:latest

# Besu network (validators, explorer, monitoring); versions pinned in its .env
BLOCKCHAIN_DIR             := ../blockchain/quorum-test-network
BLOCKCHAIN_COMPOSE         := $(BLOCKCHAIN_DIR)/docker-compose.yml
QUORUM_EXPLORER_VERSION    := $(shell sed -n 's/^QUORUM_EXPLORER_VERSION=//p' $(BLOCKCHAIN_DIR)/.env)
IMG_QUORUM_EXPLORER        := quorum-explorer:$(QUORUM_EXPLORER_VERSION)
QUORUM_EXPLORER_SOURCE     := https://github.com/Consensys-inc-archive/quorum-explorer.git

# --- Main Command ---

# Default target: Builds all in parallel and prints summary on success
all:
	@echo ">> 🚀 Starting parallel build for all images..."
	@$(MAKE) -j blockchain quorum-explorer attestation-sidecar docker-stats-collector gazebo-server gazebo-gui robot-simulation hardhat secaas
	@echo ""
	@echo "========================================================"
	@echo "   ✅  ALL IMAGES BUILT SUCCESSFULLY"
	@echo "========================================================"

# Pull the pinned upstream images ahead of start.sh (the explorer is built below).
blockchain:
	@echo ">> 📥 Pulling blockchain images ($(BLOCKCHAIN_COMPOSE))..."
	@docker compose -f $(BLOCKCHAIN_COMPOSE) pull --quiet

# No longer on Docker Hub: built from the archived source at the pinned commit.
quorum-explorer:
	@echo ">> 📦 Building [$(IMG_QUORUM_EXPLORER)]..."
	@docker build -t $(IMG_QUORUM_EXPLORER) \
		--build-context source=$(QUORUM_EXPLORER_SOURCE)#$(QUORUM_EXPLORER_VERSION) \
		./quorum-explorer

# Remove the specific images built by this file
clean:
	@echo ">> 🧹 Removing project images..."
	-docker rmi $(IMG_ATTESTATION_SIDECAR) $(IMG_DOCKER_STATS_COLLECTOR) $(IMG_GAZEBO_SERVER) $(IMG_GAZEBO_GUI) $(IMG_ROBOT_SIMULATION) $(IMG_HARDHAT) $(IMG_SECAAS) $(IMG_QUORUM_EXPLORER)

# --- Individual Build Rules ---

attestation-sidecar:
	@echo ">> 📦 Building [$(IMG_ATTESTATION_SIDECAR)]..."
	@docker build -t $(IMG_ATTESTATION_SIDECAR) ./attestation-sidecar

docker-stats-collector:
	@echo ">> 📦 Building [$(IMG_DOCKER_STATS_COLLECTOR)]..."
	@docker build -t $(IMG_DOCKER_STATS_COLLECTOR) ./docker-stats-collector

# The worlds and models come from turtlebot3_gazebo in robot-simulation/.
gazebo-server:
	@echo ">> 📦 Building [$(IMG_GAZEBO_SERVER)]..."
	@docker build -t $(IMG_GAZEBO_SERVER) \
		--build-context turtlebot3_gazebo=./robot-simulation/src/turtlebot3_simulations/turtlebot3_gazebo \
		./gazebo-server

gazebo-gui:
	@echo ">> 📦 Building [$(IMG_GAZEBO_GUI)]..."
	@docker build -t $(IMG_GAZEBO_GUI) ./gazebo-gui

robot-simulation:
	@echo ">> 📦 Building [$(IMG_ROBOT_SIMULATION)]..."
	@docker build -t $(IMG_ROBOT_SIMULATION) ./robot-simulation

hardhat:
	@echo ">> 📦 Building [$(IMG_HARDHAT)]..."
	@docker build -t $(IMG_HARDHAT) ./hardhat

secaas:
	@echo ">> 📦 Building [$(IMG_SECAAS)]..."
	@docker build -t $(IMG_SECAAS) ./secaas


orchestrator:
	@echo ">> 📦 Building [$(IMG_ORCHESTRATOR)]..."
	@docker build -t $(IMG_ORCHESTRATOR) ./orchestrator
