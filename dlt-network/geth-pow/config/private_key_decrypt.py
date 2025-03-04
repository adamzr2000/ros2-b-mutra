from web3.auto import w3
import os
import binascii

# Define the base directory path
base_dir = "./"  # Change this if necessary

# Define the password to decrypt the keystore files
password = "desire6g2024;"

# Function to decrypt and return private keys from a keystore file
def decrypt_and_get_private_key(keyfile_path):
    with open(keyfile_path) as keyfile:
        encrypted_key = keyfile.read()
        private_key = w3.eth.account.decrypt(encrypted_key, password)
    return binascii.b2a_hex(private_key).decode('utf-8')

# Define the consolidated .env file path
env_file_path = os.path.join(base_dir, ".env")

# Find and decrypt private keys in all keystore directories and write to the consolidated .env file
for root, dirs, files in os.walk(base_dir):
    for dir in dirs:
        if dir.startswith("node") and os.path.exists(os.path.join(root, dir, "keystore")):
            node_number = dir.replace("node", "")
            keystore_dir = os.path.join(root, dir, "keystore")
            
            # Decrypt each keystore file and append private keys to the .env file
            for filename in os.listdir(keystore_dir):
                if filename.startswith("UTC--"):
                    keyfile_path = os.path.join(keystore_dir, filename)
                    private_key_hex = decrypt_and_get_private_key(keyfile_path)
                    with open(env_file_path, "a") as env_file:
                        env_file.write(f"PRIVATE_KEY_NODE_{node_number}={private_key_hex}\n")
                    # print(f"Private key for {dir} written to {env_file_path}")

# print("Private keys have been written to the consolidated environment file.")
