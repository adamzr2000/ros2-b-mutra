import json
import os
import hashlib
import math
import uuid
from dotenv import dotenv_values

# Load blockchain and contract environment configuration
geth_config_env_file = dotenv_values('./dlt-network/geth-poa/.env')
contract_env_file = dotenv_values('./smart-contracts/smart-contract.env')

# Extract contract address
contract_address = contract_env_file.get('CONTRACT_ADDRESS')

# Function to retrieve node credentials dynamically
def get_node_credentials(node_index):
    return {
        "eth_address": geth_config_env_file.get(f'ETHERBASE_NODE_{node_index}'),
        "private_key": geth_config_env_file.get(f'PRIVATE_KEY_NODE_{node_index}'),
        "eth_node": geth_config_env_file.get(f'WS_NODE_{node_index}_URL')
    }

# Function to generate a fake reference signature
def generate_fake_hash(data: str) -> str:
    return hashlib.sha256(data.encode('utf-8')).hexdigest()

# Function to create an agent JSON config
def create_agent_config(agent_index, node_index, output_directory):
    credentials = get_node_credentials(node_index)
    
    agent_data = {
        "name": f"robot{agent_index}",
        "uuid": str(uuid.uuid4()),  # Generate a proper random UUID
        "eth_address": credentials["eth_address"],
        "private_key": credentials["private_key"],
        "eth_node": credentials["eth_node"],
        "contract_address": contract_address,
        "ref_signature": generate_fake_hash(f"robot{agent_index}")
    }

    filename = f"Agent_{agent_index}.json"
    filepath = os.path.join(output_directory, filename)
    
    with open(filepath, "w") as file:
        json.dump(agent_data, file, indent=4)
    
    print(f"Created {filename} with Node {node_index} credentials.")

# Function to create secaas JSON config
def create_secaas_config(output_directory):
    node_index = 1  # Always use Node 1 for secaas
    credentials = get_node_credentials(node_index)
    
    secaas_data = {
        "name": "secaas",
        "eth_address": credentials["eth_address"],
        "private_key": credentials["private_key"],
        "eth_node": credentials["eth_node"],
        "contract_address": contract_address,
    }

    filename = "Secaas.json"
    filepath = os.path.join(output_directory, filename)
    
    with open(filepath, "w") as file:
        json.dump(secaas_data, file, indent=4)
    
    print(f"Created {filename} with Node {node_index} credentials.")

# Function to calculate robot positions in a grid to prevent collisions
def calculate_position(agent_index):
    spacing = 5  # Adjust to control the minimum distance between robots
    grid_size = math.ceil(math.sqrt(agent_index))  # Create a square grid layout
    row = (agent_index - 1) // grid_size
    col = (agent_index - 1) % grid_size
    x_pose = col * spacing
    y_pose = row * -spacing  # Negative y-spacing to spread out robots
    return x_pose, y_pose

# Function to generate docker-compose.yml (saved in `./`)
def generate_docker_compose(num_agents):
    compose_content = '''version: '3'

x-common-commands:
  gzserver_command: &gzserver_command >
    bash -c "
    source /home/agent/ros2_ws/install/setup.bash && \
    ros2 launch turtlebot3_gazebo gazebo_server.launch.py verbose:=true \
    "

  robot_entrypoint: &robot_entrypoint >
    bash -c "
    ./start_turtlebot.sh
    "

services:

  gazebo-server:
    image: ros2-agent:latest
    container_name: gazebo-server
    hostname: gazebo-server
    environment:
      - ROS_DOMAIN_ID=42   
      - GAZEBO_AUDIO_DEVICE=none
    ports:
      - "11345:11345"
    tty: true
    stdin_open: true
    command: *gzserver_command
    networks:
      - dlt_network

  gazebo-client:
    image: gazebo-vnc:latest
    container_name: gazebo-client
    hostname: gazebo-client
    environment:
      - ROS_DOMAIN_ID=42
      - GAZEBO_MASTER_URI=http://gazebo-server:11345    
    depends_on:
      - gazebo-server    
    ports:
      - "6080:80"
    security_opt:
      - seccomp=unconfined
    restart: unless-stopped
    networks:
      - dlt_network

  secaas:
    image: ros2-agent:latest
    container_name: secaas
    hostname: secaas
    volumes:
      - "./config/:/home/agent/config/"
      - "./smart-contracts:/home/agent/smart-contracts"
      - "./dockerfiles/turtlebot3/scripts/mas_mutual_attestation.py:/home/agent/scripts/mas_mutual_attestation.py"
    stdin_open: true
    tty: true    
    networks:
      - dlt_network
'''

    for i in range(1, num_agents + 1):
        agent_filename = f"Agent_{i}.json"
        agent_name = f"robot{i}"
        x_pose, y_pose = calculate_position(i)

        compose_content += f'''  {agent_name}:
    image: ros2-agent:latest
    container_name: {agent_name}
    hostname: {agent_name}
    volumes:
      - "./config/{agent_filename}:/home/agent/config/{agent_filename}"
      - "./smart-contracts:/home/agent/smart-contracts"
      - "./dockerfiles/turtlebot3/ros2-scripts:/home/agent/ros2-scripts"
      - "./dockerfiles/turtlebot3/scripts/mas_mutual_attestation.py:/home/agent/scripts/mas_mutual_attestation.py"
    environment:
        - ROS_DOMAIN_ID=42
        - GAZEBO_MASTER_URI=http://gazebo-server:11345
        - NAMESPACE={agent_name}
        - X_POSE={x_pose}
        - Y_POSE={y_pose}
    depends_on:
      - gazebo-server
    stdin_open: true
    tty: true
    networks:
      - dlt_network
'''

    compose_content += '''
networks:
  dlt_network:
    external: true
'''

    compose_file_path = "./docker-compose.yml"  # Save in the current directory
    with open(compose_file_path, "w") as file:
        file.write(compose_content)

    print(f"Docker Compose file generated at {compose_file_path}")

# Main script execution
if __name__ == "__main__":
    output_directory = "./config"
    os.makedirs(output_directory, exist_ok=True)
    
    # Get number of agents from user input
    num_agents = int(input("Enter the number of agents: ").strip())

    create_secaas_config(output_directory)

    # Generate configurations
    for i in range(1, num_agents + 1):
        create_agent_config(i, i+1, output_directory)  # Increment node index dynamically

    # Generate Docker Compose file in `./`
    generate_docker_compose(num_agents)

    print(f"Successfully created {num_agents} agent configurations in {output_directory} and docker-compose.yml in current directory.")
