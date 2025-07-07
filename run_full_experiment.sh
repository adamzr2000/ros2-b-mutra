#!/bin/bash
set -e

# 1. Initialize the blockchain network
cd dlt-network/geth-poa
./start_geth_poa_network.sh

sleep 3

# 2. Deploy the smart contract
cd ../..
./deploy_smart_contract.sh --node-ip 10.0.1.1 --ws-port 3334 --http-port 3335

sleep 3

# 3. Start the experimental scenario (docker compose)
docker compose up -d

# 4. Run blockchain-based attestation on SECaaS (already started in the docker-compose)
# docker exec -it secaas bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant secaas"
 
# 5. Run blockchain-based attestation on Robots (already started in the docker-compose -- sidecar controller and sidecar)
#docker exec -it robot1 bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap"