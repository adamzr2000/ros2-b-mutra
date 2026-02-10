# app/internal/compute/hash.py

import subprocess
import hashlib
from app.internal.logger import info, warn, error, debug, green_text

class ComputeHashError(Exception):
    """Base exception for compute hash operations"""
    pass

def get_pid(program_name):
    try:
        pids = subprocess.check_output(["pidof", program_name]).decode().strip().split()
        pids = [int(x) for x in pids]
        return min(pids) if pids else None
    except subprocess.CalledProcessError:
        raise ComputeHashError("Program not found")

def get_text_section_size(binary_path):
    try:
        output = subprocess.check_output(["readelf", "-W", "-S", binary_path]).decode()
        for line in output.splitlines():
            if '.text' in line:
                return int(line.split()[5], 16)
        raise ComputeHashError("No .text section found")
    except Exception as e:
        raise ComputeHashError(f"Error reading text section: {str(e)}")

def get_first_rx_page_address(pid):
    try:
        with open(f"/proc/{pid}/maps", "r") as maps_file:
            for line in maps_file:
                parts = line.strip().split()
                if 'r-xp' in parts[1]:
                    address_range = parts[0].split('-')
                    return int(address_range[0], 16)
        raise ComputeHashError("No RX page found")
    except Exception as e:
        raise ComputeHashError(f"Error reading memory maps: {str(e)}")

def read_memory(pid, address, size):
    try:
        with open(f"/proc/{pid}/mem", "rb") as mem_file:
            mem_file.seek(address)
            memory_data = mem_file.read(size)

        # with open("./memory_dump.bin", 'wb') as output_file:
        #     output_file.write(memory_data)

        if not memory_data:
            raise ComputeHashError("Failed to read memory data")
        return memory_data
    except Exception as e:
        raise ComputeHashError(f"Error reading process memory: {str(e)}")

def compute_sha256(buffer):
    sha256 = hashlib.sha256()
    sha256.update(buffer)
    return sha256.hexdigest()

def compute_program_hash(program_name, text_size, offset):
    debug(f"Computing hash for process '{program_name}' with text_size={text_size}, offset={offset}")
    
    pid = get_pid(program_name)
    if pid is None:
        error(f"Process '{program_name}' not found")
        raise ComputeHashError(f"Process {program_name} not found")
    debug(f"Found PID: {pid} for process '{program_name}'")
        
    rx_address = get_first_rx_page_address(pid)
    if rx_address is None:
        error(f"Failed to get RX page address for PID {pid}")
        raise ComputeHashError(f"Failed to get RX page address for PID {pid}")
    debug(f"Found RX page address: 0x{rx_address:x} for PID {pid}")
    
    # Ensure offset is an integer
    try:
        begin_offset = int(offset) if offset is not None else 0
        debug(f"Using offset: {begin_offset}")
    except (ValueError, TypeError):
        error(f"Invalid offset value: {offset}")
        raise ComputeHashError(f"Invalid offset value: {offset}")
    
    begin_address = rx_address + begin_offset
    debug(f"Calculated begin_address: 0x{begin_address:x}")
    
    # Ensure text_size is an integer
    try:
        text_size = int(text_size) if text_size is not None else 0
        debug(f"Using text_size: {text_size}")
    except (ValueError, TypeError):
        error(f"Invalid text_section_size value: {text_size}")
        raise ComputeHashError(f"Invalid text_section_size value: {text_size}")
    
    if text_size <= 0:
        error(f"Invalid text_section_size: {text_size}")
        raise ComputeHashError(f"Invalid text_section_size: {text_size}")
    
    debug(f"Reading {text_size} bytes of memory from address 0x{begin_address:x} for PID {pid}")
    memory_content = read_memory(pid, begin_address, text_size)
    hash_result = compute_sha256(memory_content)
    debug(f"Computed hash: {hash_result[:8]}...{hash_result[-8:]}")
    return hash_result
