# utils.py
import os
import json
import csv

def get_agent_by_name(directory: str, target_name: str):
    
    """
    Searches for an agent's information based on the given name.

    Args:
        directory (str): The directory containing the agent JSON files.
        target_uuid (str): The UUID to search for.

    Returns:
        dict: The agent's information if found, otherwise None.
    """
    # Normalize target name (convert to lowercase and remove dashes)
    target_name_normalized = target_name.replace("-", "").lower()

    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)

            with open(file_path, "r") as json_file:
                try:
                    data = json.load(json_file)
                    
                    # Normalize name from JSON file
                    json_name = str(data.get("name", "")).replace("-", "").lower()

                    if json_name == target_name_normalized:
                        return data  # Return full agent info as a dictionary
                
                except (json.JSONDecodeError, KeyError):
                    print(f"Warning: Failed to parse {file_path} or missing key.")

    return None  # Return None if no match is found

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


def export_attestation_result_csv(agent_name, attestation_id, role, result, timestamps):
    """
    Export absolute timestamps for attestation steps to /experiments/<agent_name>-<role>.csv
    with 't_' prefix for step columns.
    """
    # Ensure the output directory exists
    os.makedirs("/experiments", exist_ok=True)

    filename = f"/experiments/{agent_name}-{role}.csv"

    ROLE_STEPS = {
        "prover": ["start", "signature_prepared", "signature_sent", "result_received"],
        "verifier": ["start", "evaluation_ready_received", "result_sent"]
    }

    steps = ROLE_STEPS[role]
    headers = ["attestation_id", "role", "result"] + [f"t_{step}" for step in steps]

    # Use raw timestamps (Unix epoch seconds)
    timestamp_values = [round(timestamps.get(step, 0), 3) if step in timestamps else "" for step in steps]

    row = [attestation_id, role, result] + timestamp_values
    file_exists = os.path.isfile(filename)

    with open(filename, mode='a', newline='') as csv_file:
        writer = csv.writer(csv_file)
        if not file_exists:
            writer.writerow(headers)
        writer.writerow(row)
