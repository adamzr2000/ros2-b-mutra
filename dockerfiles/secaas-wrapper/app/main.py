from fastapi import FastAPI, File, UploadFile, Form
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy import create_engine, Column, String
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from hashlib import sha256
from elftools.elf.elffile import ELFFile
from io import BytesIO
from pydantic import BaseModel, conlist
from typing import List
from dotenv import load_dotenv
import os
import uuid
from uuid import UUID
import json
import glob
import aiofiles
import httpx
from app.log_wrapper import RedisLogger

load_dotenv()

# Initialize Redis logger
logger = RedisLogger()

SQLALCHEMY_DATABASE_URL = f"postgresql://{os.getenv('POSTGRES_USER')}:{os.getenv('POSTGRES_PASSWORD')}@{os.getenv('POSTGRES_HOST')}:{os.getenv('POSTGRES_PORT')}/{os.getenv('POSTGRES_DB')}"
engine = create_engine(SQLALCHEMY_DATABASE_URL)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

class Measure(Base):
    __tablename__ = "measures"

    id = Column(String, primary_key=True, index=True)
    agent_name = Column(String, index=True)
    sha256 = Column(String, index=True)
    cmd_name = Column(String, index=True)

Base.metadata.create_all(bind=engine)

class MeasureCreate(BaseModel):
    agentname: str
    sha256: str
    id: str
    cmdname: str

class UUIDListRequest(BaseModel):
    uuids: conlist(UUID, min_items=1)

app = FastAPI()

origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/wrapAll")
async def wrapAll():
    try:
        responses = []
        errors = []
        operation_logs = []
        
        db = SessionLocal()
        logger.log_operation('info', "Database session created")
        
        number_of_agents = int(os.getenv('NUMBER_OF_AGENTS', 3))
        logger.log_operation('info', f"Processing {number_of_agents} agents")
        
        # Validate all agent files exist before processing
        agent_files = {}
        for idx in range(1, number_of_agents + 1):
            agent_file = os.getenv(f'AGENT_FILE{idx}')
            if not agent_file:
                error_msg = f"AGENT_FILE{idx} environment variable not set"
                logger.log_error(error_msg, {'agent_index': idx})
                errors.append({"agent": idx, "error": error_msg})
                continue
            agent_files[idx] = agent_file
            logger.log_operation('info', f"Validated agent {idx} file: {agent_file}")
        
        if not agent_files:
            logger.log_error("No valid agent files found")
            return {
                "message": "No valid agent files found",
                "errors": errors
            }
        
        for idx, filepath in agent_files.items():
            try:
                logger.log_operation('info', f"Processing agent {idx}/{number_of_agents}")
                logger.log_operation('info', f"Opening file: {filepath}")
                
                with open(f"/binaryfiles/{filepath}", 'rb') as f:
                    elffile = ELFFile(f)
                    
                    text_section = elffile.get_section_by_name('.text')
                    if not text_section:
                        error_msg = f"No .text section found in agent {idx}"
                        logger.log_error(error_msg, {'agent_index': idx})
                        errors.append({"agent": idx, "error": error_msg})
                        continue

                    text_data = text_section.data()
                    text_size = len(text_data)
                    hash_sha256 = sha256(text_data).hexdigest()
                    logger.log_operation('info', f"Calculated SHA256 hash for agent {idx}")
                    
                    file_id = str(uuid.uuid4())
                    
                    # Save to database
                    measure = Measure(
                        id=file_id,
                        agent_name=f"agent{idx}",
                        sha256=hash_sha256,
                        cmd_name=filepath
                    )
                    db.add(measure)
                    logger.log_operation('info', f"Database entry created for agent {idx}")
                    
                    # Store agent file_id in Redis
                    agent_data = {
                        "file_id": file_id,
                        "sha256": hash_sha256,
                        "cmd_name": filepath,
                        "agent_name": f"agent{idx}"
                    }
                    logger.redis_client.rpush("wrapped_agents", json.dumps(agent_data))
                    logger.log_operation('info', f"Agent {idx} data added to wrapped_agents list in Redis")
                    # Create response
                    response_dict = {
                        "agent": idx,
                        "id": file_id,
                        "agent_name": f"agent{idx}",
                        "cmd_name": filepath,
                        "text_section_size": text_size,
                        "sha256": hash_sha256,
                        "status": "success"
                    }
                    
                    # Save JSON response
                    json_path = f"/jsonfiles/agent{idx}.json"
                    with open(json_path, 'w', encoding='utf-8') as f:
                        json.dump(response_dict, f, ensure_ascii=False, indent=4)
                    logger.log_operation('info', f"JSON file created: {json_path}")
                    
                    responses.append(response_dict)
                    logger.log_operation('info', f"Successfully processed agent {idx}")
                    
            except Exception as e:
                error_msg = f"Error processing agent {idx}: {str(e)}"
                logger.log_error(error_msg, {'agent_index': idx})
                errors.append({"agent": idx, "error": error_msg})
                continue
        
        try:
            logger.log_operation('info', "Committing database changes")
            db.commit()
            logger.log_operation('info', "Database changes committed successfully")
        except Exception as e:
            db.rollback()
            error_msg = f"Database commit failed: {str(e)}"
            logger.log_error(error_msg)
            logger.log_operation('info', "Database changes rolled back")
            errors.append({"database": "commit_error", "error": error_msg})
        
        return {
            "processed": responses,
            "errors": errors,
            "operation_logs": [],
            "summary": {
                "total_agents": number_of_agents,
                "successful": len(responses),
                "failed": len(errors)
            }
        }

    except Exception as e:
        if 'db' in locals():
            db.rollback()
            if 'operation_logs' in locals():
                logger.log_operation('info', "Database transaction rolled back due to error")
        error_response = {
            "message": f"Fatal error in wrapAll: {str(e)}",
            "status": "error",
            "operation_logs": []
        }
        return error_response
        
    finally:
        if 'db' in locals():
            if 'operation_logs' in locals():
                logger.log_operation('info', "Closing database connection")
            db.close()

@app.get("/wrapAlldmutra")
async def wrapAlldmutra():
    try:
        responses = []
        errors = []
        operation_logs = []
        
        db = SessionLocal()
        logger.log_operation('info', "Database session created for dmutra agents")
        
        # Define the specific dmutra agents to process
        dmutra_agents = {
            1: "agent_dmutra1",
            2: "agent_dmutra2"
        }
        
        logger.log_operation('info', f"Processing {len(dmutra_agents)} dmutra agents")
        
        # Validate all agent files exist before processing
        agent_files = {}
        for idx, agent_name in dmutra_agents.items():
            agent_file = os.getenv(f'AGENT_FILE{idx}')
            if not agent_file:
                error_msg = f"AGENT_FILE{idx} environment variable not set for {agent_name}"
                logger.log_error(error_msg, {'agent_index': idx, 'agent_name': agent_name})
                errors.append({"agent": idx, "agent_name": agent_name, "error": error_msg})
                continue
            agent_files[idx] = agent_file
            logger.log_operation('info', f"Validated dmutra agent {idx} file: {agent_file}")
        
        if not agent_files:
            logger.log_error("No valid dmutra agent files found")
            return {
                "message": "No valid dmutra agent files found",
                "errors": errors
            }
        
        for idx, filepath in agent_files.items():
            try:
                agent_name = dmutra_agents[idx]
                logger.log_operation('info', f"Processing dmutra agent {idx}: {agent_name}")
                logger.log_operation('info', f"Opening file: {filepath}")
                
                with open(f"/binaryfiles/{filepath}", 'rb') as f:
                    elffile = ELFFile(f)
                    
                    text_section = elffile.get_section_by_name('.text')
                    if not text_section:
                        error_msg = f"No .text section found in dmutra agent {idx}: {agent_name}"
                        logger.log_error(error_msg, {'agent_index': idx, 'agent_name': agent_name})
                        errors.append({"agent": idx, "agent_name": agent_name, "error": error_msg})
                        continue

                    text_data = text_section.data()
                    text_size = len(text_data)
                    hash_sha256 = sha256(text_data).hexdigest()
                    logger.log_operation('info', f"Calculated SHA256 hash for dmutra agent {idx}: {agent_name}")
                    
                    file_id = str(uuid.uuid4())
                    
                    # Save to database
                    measure = Measure(
                        id=file_id,
                        agent_name=agent_name,
                        sha256=hash_sha256,
                        cmd_name=filepath
                    )
                    db.add(measure)
                    logger.log_operation('info', f"Database entry created for dmutra agent {idx}: {agent_name}")
                    
                    # Store agent file_id in Redis
                    agent_data = {
                        "file_id": file_id,
                        "sha256": hash_sha256,
                        "cmd_name": filepath,
                        "agent_name": agent_name
                    }
                    logger.redis_client.rpush("wrapped_agents", json.dumps(agent_data))
                    logger.log_operation('info', f"Dmutra agent {idx}: {agent_name} data added to wrapped_agents list in Redis")
                    
                    # Create response
                    response_dict = {
                        "agent": idx,
                        "id": file_id,
                        "agent_name": agent_name,
                        "cmd_name": filepath,
                        "text_section_size": text_size,
                        "sha256": hash_sha256,
                        "status": "success"
                    }
                    
                    # Save JSON response
                    json_path = f"/jsonfiles/{agent_name}.json"
                    with open(json_path, 'w', encoding='utf-8') as f:
                        json.dump(response_dict, f, ensure_ascii=False, indent=4)
                    logger.log_operation('info', f"JSON file created: {json_path}")
                    
                    responses.append(response_dict)
                    logger.log_operation('info', f"Successfully processed dmutra agent {idx}: {agent_name}")
                    
            except Exception as e:
                error_msg = f"Error processing dmutra agent {idx}: {dmutra_agents.get(idx, f'agent{idx}')} - {str(e)}"
                logger.log_error(error_msg, {'agent_index': idx, 'agent_name': dmutra_agents.get(idx, f'agent{idx}')})
                errors.append({"agent": idx, "agent_name": dmutra_agents.get(idx, f'agent{idx}'), "error": error_msg})
                continue
        
        try:
            logger.log_operation('info', "Committing database changes for dmutra agents")
            db.commit()
            logger.log_operation('info', "Database changes for dmutra agents committed successfully")
        except Exception as e:
            db.rollback()
            error_msg = f"Database commit failed for dmutra agents: {str(e)}"
            logger.log_error(error_msg)
            logger.log_operation('info', "Database changes for dmutra agents rolled back")
            errors.append({"database": "commit_error", "error": error_msg})
        
        return {
            "processed": responses,
            "errors": errors,
            "operation_logs": [],
            "summary": {
                "total_dmutra_agents": len(dmutra_agents),
                "successful": len(responses),
                "failed": len(errors)
            }
        }

    except Exception as e:
        if 'db' in locals():
            db.rollback()
            if 'operation_logs' in locals():
                logger.log_operation('info', "Database transaction rolled back due to error in dmutra agents")
        error_response = {
            "message": f"Fatal error in wrapAlldmutra: {str(e)}",
            "status": "error",
            "operation_logs": []
        }
        return error_response
        
    finally:
        if 'db' in locals():
            if 'operation_logs' in locals():
                logger.log_operation('info', "Closing database connection for dmutra agents")
            db.close()

@app.get("/wrapped-agents")
async def get_wrapped_agents():
    try:
        # Get wrapped agents from Redis
        wrapped_agents_data = logger.redis_client.lrange("wrapped_agents", 0, -1)
        
        # Parse JSON data
        agents = []
        for agent_data in wrapped_agents_data:
            agents.append(json.loads(agent_data))
            
        return {"agents": agents}
    except Exception as e:
        logger.log_error(f"Error retrieving wrapped agents: {str(e)}")
        return {"error": str(e), "agents": []}
