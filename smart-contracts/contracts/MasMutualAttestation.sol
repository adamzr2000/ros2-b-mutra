// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract MasMutualAttestation {

    enum AttestationResult {None, Failure, Success}
    enum AttestationState {Open, SecaasResponded, ProverResponded, ReadyForEvaluation, Closed}

    struct Agent {
        bytes16 uuid;
        bool registered;
        bool verified;
    }

    struct MutualAttestation {
        bytes32 id;
        address verifier;
        address prover;
        bytes32 fresh_signature;
        bytes32 ref_signature;
        AttestationState state;
        AttestationResult result;
        uint256 timestamp; // Timestamp for attestation closure
    }

    mapping(address => Agent) private agent;
    mapping(bytes32 => MutualAttestation) private attestation;
    address private currentVerifier;
    address private secaas;
    uint256 private currentAttestationIndex;  

    event AgentRegistered(address agent);
    event AgentRemoved(address agent);
    event AttestationStarted(bytes32 id);
    event SecaasResponded(bytes32 id);
    event ProverResponded(bytes32 id);
    event ReadyForEvaluation(bytes32 id);
    event AttestationCompleted(bytes32 id);
    event ChainReset();

    constructor() {
        secaas = msg.sender;
        currentVerifier = secaas;
        currentAttestationIndex = 1;
    }

    function ResetChain() public {
        require(msg.sender == secaas, "Only the SECaaS can reset the chain");

        // Reset attestation index and current verifier
        currentAttestationIndex = 1;
        currentVerifier = secaas;

        emit ChainReset();
    }    

    function RegisterAgent(bytes16 uuid) public {
        require(uuid != bytes16(0), "UUID is not valid");

        Agent storage currentAgent = agent[msg.sender];
        require(!currentAgent.registered, "Agent already registered");

        currentAgent.uuid = uuid;
        currentAgent.registered = true;
        currentAgent.verified = false;

        emit AgentRegistered(msg.sender);

        // Generate a unique attestation ID
        bytes32 id = keccak256(abi.encodePacked(msg.sender, block.timestamp, currentAttestationIndex));

        MutualAttestation storage currentAttestation = attestation[id];
        currentAttestation.id = id;
        currentAttestation.verifier = currentVerifier;
        currentAttestation.prover = msg.sender;
        currentAttestation.fresh_signature = bytes32(0);
        currentAttestation.ref_signature = bytes32(0);
        currentAttestation.state = AttestationState.Open;
        currentAttestation.result = AttestationResult.None;
        currentAttestation.timestamp = 0; 

        emit AttestationStarted(id);
        currentAttestationIndex++;
    }

    function RemoveAgent() public {
        Agent storage currentAgent = agent[msg.sender];
        require(currentAgent.registered, "Agent is not registered");
        delete agent[msg.sender];
        emit AgentRemoved(msg.sender);
    }

    function GetAgentInfo(address callAddress) public view returns (bytes16, bool, bool) {
        Agent storage currentAgent = agent[callAddress];
        require(currentAgent.registered || callAddress == secaas, "Agent is not registered");
        return (currentAgent.uuid, currentAgent.registered, currentAgent.verified);
    } 

    function GetProverUUID(bytes32 id, address callAddress) public view returns (bytes16, address) {
        require(callAddress == secaas, "You are not the SECaaS.");
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state == AttestationState.Open || currentAttestation.state == AttestationState.ProverResponded, "Attestation is closed or not exists");
        address proverAgentAddress = currentAttestation.prover;
        Agent storage currentAgent = agent[proverAgentAddress];
        return (currentAgent.uuid, msg.sender);
    }     

    function SendFreshSignaure(bytes32 id, bytes32 measurement) public returns (bool) {
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state == AttestationState.Open || currentAttestation.state == AttestationState.SecaasResponded, "Invalid state for fresh signature submission");
        require(currentAttestation.prover == msg.sender, "You are not the prover.");
        
        currentAttestation.fresh_signature = measurement;
    
        if (currentAttestation.state == AttestationState.SecaasResponded) {
            currentAttestation.state = AttestationState.ReadyForEvaluation;
            emit ReadyForEvaluation(id);
        } else {
            currentAttestation.state = AttestationState.ProverResponded;
            emit ProverResponded(id);
        }

        return true;
    }

    function SendRefSignaure(bytes32 id, bytes32 measurement) public returns (bool) {
        require(msg.sender == secaas, "Only the SECaaS can send the reference signature");
        
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state == AttestationState.Open || currentAttestation.state == AttestationState.ProverResponded, "Invalid state for SECaaS response");
        
         currentAttestation.ref_signature = measurement;
    
        if (currentAttestation.state == AttestationState.ProverResponded) {
            currentAttestation.state = AttestationState.ReadyForEvaluation;
            emit ReadyForEvaluation(id);
        } else {
            currentAttestation.state = AttestationState.SecaasResponded;
            emit SecaasResponded(id);
        }

        return true;
    }

    function GetAttestationMeasurements(bytes32 id, address callAddress) public view returns (bytes32, bytes32) {
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.verifier == callAddress, "Only the verifier can retrieve the signatures");
        require(currentAttestation.state == AttestationState.ReadyForEvaluation, "Attestation process is not completed");
        return (currentAttestation.fresh_signature, currentAttestation.ref_signature);
    }

    function GetAttestationInfo(bytes32 id) public view returns (address, address, AttestationResult, uint256) {
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state == AttestationState.Closed, "Attestation process is not closed");
        return (currentAttestation.prover, currentAttestation.verifier, currentAttestation.result, currentAttestation.timestamp);
    }

    function GetAttestationState(bytes32 id) public view returns (AttestationState) {
        return attestation[id].state;
    }

    function CloseAttestationProcess(bytes32 id, bool verified) public returns (bool) {
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.verifier == msg.sender, "Only the verifier can close the attestation process");
        require(currentAttestation.state == AttestationState.ReadyForEvaluation, "Attestation is not ready for closure");
        
        currentAttestation.state = AttestationState.Closed;
        currentAttestation.timestamp = block.timestamp; // Store closure timestamp
        Agent storage proverAgent = agent[currentAttestation.prover];
        currentAttestationIndex++;

        if (verified) {
            proverAgent.verified = true;
            currentAttestation.result = AttestationResult.Success;
            currentVerifier = currentAttestation.prover;
        } else {
            proverAgent.verified = false;
            currentAttestation.result = AttestationResult.Failure;
        }

        emit AttestationCompleted(id);
        return true;
    }

    function IsProver(bytes32 id, address callAddress) public view returns (bool) {
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state != AttestationState.Closed, "Attestation process is closed or not exists");
        return currentAttestation.prover == callAddress;
    }

    function IsVerifier(bytes32 id, address callAddress) public view returns (bool) {
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state != AttestationState.Closed, "Attestation process is closed or not exists");
        return currentAttestation.verifier == callAddress;
    }

}
