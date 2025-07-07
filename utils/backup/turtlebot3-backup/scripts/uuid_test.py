import json
import uuid
import os

def get_agent_by_uuid(directory: str, target_uuid: str):
    """
    Searches for an agent's information based on the given UUID.

    Args:
        directory (str): The directory containing the agent JSON files.
        target_uuid (str): The UUID to search for.

    Returns:
        dict: The agent's information if found, otherwise None.
    """
    # Normalize target UUID (convert to lowercase and remove dashes)
    target_uuid_normalized = target_uuid.replace("-", "").lower()

    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)

            with open(file_path, "r") as json_file:
                try:
                    data = json.load(json_file)
                    
                    # Normalize UUID from JSON file
                    json_uuid = str(data.get("uuid", "")).replace("-", "").lower()

                    if json_uuid == target_uuid_normalized:
                        return data  # Return full agent info as a dictionary
                
                except (json.JSONDecodeError, KeyError):
                    print(f"Warning: Failed to parse {file_path} or missing key.")

    return None  # Return None if no match is found

# Define the directory containing the agent JSON files
agents_directory = "/home/agent/config"

# UUID you want to find
uuid_to_find_str_v1 = "65c17490-c173-4280-bd76-50b94087c776"
print(f"UUID string format v1: {uuid_to_find_str_v1}")

# Convert the UUID string to a UUID object
uuid_obj = uuid.UUID(uuid_to_find_str_v1)
print(f"UUID object format: {uuid_obj}")

# Convert the UUID object to bytes
uuid_bytes16 = uuid_obj.bytes
print(f"UUID bytes format: {uuid_bytes16}")

# Convert the UUID bytes to a string
uuid_to_find_str = uuid.UUID(bytes=uuid_bytes16).hex
print(f"UUID string format: {uuid_to_find_str}")

# Call the function
agent_info = get_agent_by_uuid(agents_directory, uuid_to_find_str)

# Print the result
if agent_info:
    print("Agent found:", agent_info)
else:
    print("Agent not found.")
