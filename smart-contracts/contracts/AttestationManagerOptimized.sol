// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract AttestationManagerOptimized {

    enum AttestationResult {None, Failure, Success}

    enum AttestationState {Open, ReadyForEvaluation, Closed}

    struct Agent {
        string name;
        bool registered;
    }

    struct MutualAttestation {
        bytes32 id;
        address verifier;
        address prover;
        bytes32 fresh_signature;
        bytes32 ref_signature;
        AttestationState state;
        AttestationResult result;
        uint256 timestamp;
    }

    address private secaas;
    address[] private participants;

    // VRP state
    uint256 private vrp;
    address private currentVerifier;
    uint256 private verifierAttestationCount;

    mapping(address => Agent) private agent;
    mapping(bytes32 => MutualAttestation) private attestation;
    mapping(address => uint256) private indexOf;

    event AgentRegistered(address agent);
    event AgentRemoved(address agent);
    event AttestationStarted(bytes32 id);
    event ReadyForEvaluation(bytes32 id);
    event AttestationCompleted(bytes32 indexed id, bool verified);
    event VerifierRotated(address indexed newVerifier);
    event ChainReset();

    constructor(uint256 _vrp) {
        secaas = msg.sender;
        participants.push(secaas);
        indexOf[secaas] = 0;
        vrp = _vrp == 0 ? 1 : _vrp;
        currentVerifier = secaas;
        verifierAttestationCount = 0;
    }

    function ResetChain() public {
        require(msg.sender == secaas, "Only the SECaaS can reset the chain");
        currentVerifier = secaas;
        verifierAttestationCount = 0;
        emit ChainReset();
    }

    function RegisterAgent(string memory name) public {
        require(bytes(name).length > 0, "Name is not valid");
        Agent storage currentAgent = agent[msg.sender];
        currentAgent.name = name;
        currentAgent.registered = true;
        indexOf[msg.sender] = participants.length;
        participants.push(msg.sender);
        emit AgentRegistered(msg.sender);
    }

    function RemoveAgent() public {
        Agent storage currentAgent = agent[msg.sender];
        require(currentAgent.registered, "RemoveAgent : Agent is not registered");

        // If the departing agent is the current verifier, fall back to secaas
        if (msg.sender == currentVerifier) {
            currentVerifier = secaas;
            verifierAttestationCount = 0;
        }

        uint idx = indexOf[msg.sender];
        address last = participants[participants.length - 1];
        participants[idx] = last;
        indexOf[last] = idx;
        participants.pop();

        delete indexOf[msg.sender];
        delete agent[msg.sender];
        emit AgentRemoved(msg.sender);
    }

    // Returns currentVerifier unless prover == currentVerifier, in which case falls back to secaas.
    function _assignVerifier(address prover) internal view returns (address) {
        if (currentVerifier != prover) {
            return currentVerifier;
        }
        return secaas;
    }

    // The prover of the VRP-boundary attestation becomes the next verifier.
    function _rotateVerifier(address prover) internal {
        currentVerifier = prover;
        verifierAttestationCount = 0;
        emit VerifierRotated(prover);
    }

    function GetAgentInfo(address agentAddress, address callAddress) public view returns (string memory, bool) {
        Agent storage currentAgent = agent[callAddress];
        Agent storage agentToFind = agent[agentAddress];
        require(currentAgent.registered || callAddress == secaas, "GetAgentInfo : Agent is not registered");
        return (agentToFind.name, agentToFind.registered);
    }

    function SendEvidence(bytes32 id, bytes32 freshMeasurement) public {
        Agent storage currentAgent = agent[msg.sender];
        require(currentAgent.registered, "SendEvidence : Agent is not registered");

        MutualAttestation storage currentAttestation = attestation[id];
        currentAttestation.id = id;
        currentAttestation.verifier = _assignVerifier(msg.sender);
        currentAttestation.prover = msg.sender;
        currentAttestation.fresh_signature = freshMeasurement;
        currentAttestation.ref_signature = bytes32(0);
        currentAttestation.state = AttestationState.Open;
        currentAttestation.result = AttestationResult.None;
        currentAttestation.timestamp = 0;

        emit AttestationStarted(id);
    }

    function GetProverAddress(bytes32 id, address callAddress) public view returns (address) {
        require(callAddress == secaas, "You are not the SECaaS.");
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state == AttestationState.Open, "Attestation is closed or not exists");
        return currentAttestation.prover;
    }

    function SendRefSignature(bytes32 id, bytes32 refMeasurement) public returns (bool) {
        require(msg.sender == secaas, "Only the SECaaS can send the reference signature");

        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state == AttestationState.Open, "Invalid state for SECaaS response");

        currentAttestation.ref_signature = refMeasurement;
        currentAttestation.state = AttestationState.ReadyForEvaluation;

        emit ReadyForEvaluation(id);
        return true;
    }

    function GetAttestationSignatures(bytes32 id, address callAddress) public view returns (bytes32, bytes32) {
        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.verifier == callAddress, "Only the verifier can retrieve the signatures");
        require(
            currentAttestation.state == AttestationState.ReadyForEvaluation ||
            currentAttestation.verifier == secaas,
            "Attestation not ready for evaluation"
        );
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

        verifierAttestationCount++;
        bool vrpBoundary = verifierAttestationCount >= vrp;
        bool shouldStore = !verified || vrpBoundary;

        AttestationResult res = verified ? AttestationResult.Success : AttestationResult.Failure;

        if (shouldStore) {
            address prover = currentAttestation.prover;
            currentAttestation.state     = AttestationState.Closed;
            currentAttestation.timestamp = block.timestamp;
            currentAttestation.result    = res;
            emit AttestationCompleted(id, verified);
            if (vrpBoundary) {
                _rotateVerifier(prover);
            }
        } else {
            // Positive mid-window: delete the full struct for maximum gas refund.
            // The prover detects the result via the emitted event (WatchAttestationCompleted)
            // instead of polling GetAttestationState, so no state needs to remain on-chain.
            delete attestation[id];
            emit AttestationCompleted(id, true);
        }

        return true;
    }

    function ResolveAttestationSECaaS(bytes32 id, bool verified) public {
        require(msg.sender == secaas, "Only the SECaaS can resolve attestations");

        MutualAttestation storage currentAttestation = attestation[id];
        require(currentAttestation.state == AttestationState.Open, "Not in Open state");
        require(currentAttestation.verifier == secaas, "SECaaS not elected as verifier");

        verifierAttestationCount++;
        bool vrpBoundary = verifierAttestationCount >= vrp;
        bool shouldStore = !verified || vrpBoundary;

        AttestationResult res = verified ? AttestationResult.Success : AttestationResult.Failure;

        if (shouldStore) {
            address prover = currentAttestation.prover;
            currentAttestation.state     = AttestationState.Closed;
            currentAttestation.timestamp = block.timestamp;
            currentAttestation.result    = res;
            emit AttestationCompleted(id, verified);
            if (vrpBoundary) {
                _rotateVerifier(prover);
            }
        } else {
            delete attestation[id];
            emit AttestationCompleted(id, true);
        }
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

    function IsRegistered(address callAddress) public view returns (bool) {
        return agent[callAddress].registered;
    }

    // Read-only protocol parameter — useful for monitoring and audit.
    function GetVRP() public view returns (uint256) {
        return vrp;
    }

    // Current verifier address — already inferable from VerifierRotated events,
    // exposed here for convenience of monitoring tools.
    function GetCurrentVerifier() public view returns (address) {
        return currentVerifier;
    }
}
