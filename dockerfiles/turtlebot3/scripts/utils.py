# utils.py
import os
import json

def find_agent_json(directory: str):
    """
    Finds the path of the first JSON file with the pattern 'Agent_X.json' in the specified directory.
    
    Args:
        directory (str): The directory to search for the JSON file.
    
    Returns:
        str: The path of the found JSON file, or None if no file matching the pattern is found.
    """
    for filename in os.listdir(directory):
        if filename.startswith("Agent_") and filename.endswith(".json"):
            return os.path.join(directory, filename)
    return None

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