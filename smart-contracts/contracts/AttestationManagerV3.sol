// SPDX-License-Identifier: MIT
pragma solidity 0.8.28;

contract AttestationManagerV3 {

    enum AttestationResult {None, Failure, Success}

    enum AttestationState {Open, ReadyForEvaluation, Closed}

    struct Agent {
        string name;
        bool registered;
    }

    // A juror's vote shares the slot of its address.
    struct Juror {
        address account;
        uint8 vote; // NO_VOTE, AGAINST or FOR
    }

    // Fields are ordered so that a round fits in 3 storage slots (plus its jury).
    struct MutualAttestation {
        // slot 0
        address verifier;
        AttestationState state;
        AttestationResult result;
        bool juryDrawn;          // the jury is drawn and frozen at the first vote
        uint8 votesFor;
        uint8 votesAgainst;
        uint8 juryCount;         // jurors to draw, 0 when the SECaaS decides
        uint16 iterCount;        // IterQ depth K: freshMeasurement folds K measurements;
                                 // the verifier folds the reference K times to compare
        uint16 participantCount; // participants.length at the evidence
        uint8 boundAge;          // seedBlock minus the block whose hash the evidence is bound to
        // slot 1
        address prover;
        uint64 seedBlock;        // block holding the evidence: its hash seeds the jury draw
        uint32 flipSeqAt;        // flipSeq at the evidence: the draw uses the pool of then
        // slot 2
        bytes32 freshMeasurement;
        Juror[] jury;
    }

    // Eligibility of an agent, in one slot. slot is the index in eligible plus
    // one, 0 when absent. Each change of eligibility gets the next flipSeq;
    // the numbers of the last two changes tell the agent's eligibility at an
    // earlier evidence, unless it changed twice since.
    struct Eligibility {
        uint32 slot;
        uint32 flippedAt;
        uint32 flippedBefore;
    }

    uint8 private constant NO_VOTE = 0;
    uint8 private constant AGAINST = 1;
    uint8 private constant FOR = 2;

    // IterQ bound: rejects accidental/malicious huge K that would burn
    // verifier CPU. 10000 covers 10000 * SSP ~= 55h at SSP=20s.
    uint32 public constant MAX_ITER_COUNT = 10000;

    address private immutable secaas;

    // Verifier jury, fixed at deployment: jurySize jurors are drawn per cycle,
    // juryThreshold matching votes close it, and after juryTimeoutBlocks blocks
    // without a close the cycle goes to the SECaaS.
    uint256 private immutable jurySize;
    uint256 private immutable juryThreshold;
    uint256 public immutable juryTimeoutBlocks;

    // Evidence is bound to the hash of a block at most freshnessBlocks old,
    // newer than the one of the prover's last round, so that a rolling hash
    // cannot be submitted again later.
    uint256 public immutable freshnessBlocks;

    address[] private participants;
    mapping(address => uint256) private indexOf;
    mapping(address => Agent) private agent;
    mapping(bytes32 => MutualAttestation) private attestation;

    // Reference anchoring. referenceOf[agent] is written by the admins only,
    // never by the SECaaS. It holds SHA256(h_ref || salt_agent), so reading
    // it does not reveal which binary the agent runs.
    mapping(address => bool) private isAdmin;
    uint256 private immutable adminThreshold;
    mapping(address => bytes32) private referenceOf;
    mapping(address => uint256) private referenceVersion;
    mapping(bytes32 => mapping(address => bool)) private approvedBy;
    mapping(bytes32 => uint256) private approvalCount;
    uint256 private resetVersion; // approvals of a reset are bound to it, so they cannot be replayed

    // Agents whose last attestation succeeded: the pool the jury is drawn from.
    address[] private eligible;
    mapping(address => Eligibility) private eligibility;
    uint32 private flipSeq;

    // Last round of each prover. A prover may open a new round only once
    // this one is closed or with the SECaaS, so it cannot draw several juries
    // for the same state and keep the last verdict.
    mapping(address => bytes32) private lastRound;

    event AgentRegistered(address agent);
    event AgentRemoved(address agent);
    event AttestationStarted(bytes32 id, uint32 iterCount);
    event ReadyForEvaluation(bytes32 id);
    event AttestationCompleted(bytes32 id, address prover, bool success);
    event VoteCast(bytes32 indexed id, address indexed juror, bool verdict);
    event JurorDissent(bytes32 indexed id, address indexed juror);
    event JurorAbsent(bytes32 indexed id, address indexed juror);
    event ChainReset();
    event ResetApproved(address indexed admin, uint256 version);
    // The reference values stay out of the events: only who approved, and versions.
    event ReferenceApproved(address indexed agent, address indexed admin, uint256 version); // version approved on
    event ReferenceSet(address indexed agent, uint256 version);                             // new version

    constructor(
        address[] memory admins, uint256 threshold, uint256 size, uint256 quorum, uint256 timeoutBlocks, uint256 freshness
    ) {
        secaas = msg.sender;

        require(threshold >= 1 && threshold <= admins.length, "Invalid admin threshold");
        for (uint256 i = 0; i < admins.length; i++) {
            require(admins[i] != address(0) && admins[i] != msg.sender, "Admin must differ from the SECaaS");
            require(!isAdmin[admins[i]], "Duplicate admin");
            isAdmin[admins[i]] = true;
        }
        adminThreshold = threshold;

        require(size <= type(uint8).max, "Invalid jury size");
        require(quorum >= 1 && quorum <= size, "Invalid jury threshold");
        require(timeoutBlocks >= 1, "Invalid jury timeout");
        jurySize = size;
        juryThreshold = quorum;
        juryTimeoutBlocks = timeoutBlocks;

        // The age of the evidence is stored in 8 bits (boundAge).
        require(freshness >= 1 && freshness <= type(uint8).max, "Invalid freshness window");
        freshnessBlocks = freshness;
    }

    // ---------------------------------------------------------------- admins

    // An admin approves newReference for an agent. The reference changes once
    // adminThreshold distinct admins have approved the same value. Approvals
    // are bound to the current version, so they cannot be replayed later.
    function SetReference(address agentAddress, bytes32 newReference) external {
        require(isAdmin[msg.sender], "SetReference : caller is not an admin");
        require(newReference != bytes32(0), "SetReference : empty reference");

        bytes32 proposal = keccak256(abi.encode(agentAddress, newReference, referenceVersion[agentAddress]));
        require(!approvedBy[proposal][msg.sender], "SetReference : already approved by this admin");
        approvedBy[proposal][msg.sender] = true;
        emit ReferenceApproved(agentAddress, msg.sender, referenceVersion[agentAddress]);
        if (++approvalCount[proposal] < adminThreshold) return;

        referenceOf[agentAddress] = newReference;
        emit ReferenceSet(agentAddress, ++referenceVersion[agentAddress]);
    }

    // The admins reset the chain together: the reset applies once
    // adminThreshold distinct admins have approved it. The SECaaS cannot, so
    // it can no longer hand itself the verifier role.
    function ResetChain() external {
        require(isAdmin[msg.sender], "ResetChain : caller is not an admin");
        bytes32 proposal = keccak256(abi.encode("ResetChain", resetVersion));
        require(!approvedBy[proposal][msg.sender], "ResetChain : already approved by this admin");
        approvedBy[proposal][msg.sender] = true;
        emit ResetApproved(msg.sender, resetVersion);
        if (++approvalCount[proposal] < adminThreshold) return;

        resetVersion++;
        while (eligible.length > 0) _setEligible(eligible[eligible.length - 1], false);
        emit ChainReset();
    }

    // ---------------------------------------------------------------- agents

    function RegisterAgent(string calldata name) external {
        require(bytes(name).length > 0, "Name is not valid");
        Agent storage me = agent[msg.sender];
        // A second entry in participants would double the agent's chance to be drawn.
        require(!me.registered, "RegisterAgent : already registered");
        require(participants.length < type(uint16).max, "RegisterAgent : too many agents");
        me.name = name;
        me.registered = true;
        indexOf[msg.sender] = participants.length;
        participants.push(msg.sender);
        emit AgentRegistered(msg.sender);
    }

    function RemoveAgent() external {
        require(agent[msg.sender].registered, "RemoveAgent : Agent is not registered");
        _setEligible(msg.sender, false);

        uint256 idx = indexOf[msg.sender];
        address last = participants[participants.length - 1];
        participants[idx] = last;
        indexOf[last] = idx;
        participants.pop();

        delete indexOf[msg.sender];
        delete agent[msg.sender];
        emit AgentRemoved(msg.sender);
    }

    // ---------------------------------------------------------------- rounds

    // The prover binds each measurement to the hash of block boundBlock, which
    // it read before measuring. A rolling hash from an earlier round is bound
    // to an older block: rejected here, or failing verification if submitted
    // with a newer boundBlock.
    function Prove(bytes32 id, bytes32 freshMeasurement, uint32 iterCount, uint64 boundBlock) external {
        require(agent[msg.sender].registered, "Prove : Agent is not registered");
        require(iterCount >= 1 && iterCount <= MAX_ITER_COUNT, "Prove : iterCount out of range");
        require(referenceOf[msg.sender] != bytes32(0), "Prove : no reference anchored for this agent");
        // lastRound is 0 for an agent that never proved, so id 0 cannot be a round.
        require(id != bytes32(0), "Prove : empty id");
        MutualAttestation storage last = attestation[lastRound[msg.sender]];
        require(last.prover == address(0) || last.state == AttestationState.Closed || last.verifier == secaas,
                "Prove : prover has an open round");
        require(boundBlock < block.number && block.number - boundBlock <= freshnessBlocks,
                "Prove : evidence outside the freshness window");
        require(boundBlock > _boundBlock(last), "Prove : evidence not newer than the last round");
        MutualAttestation storage a = attestation[id];
        // A reused id would overwrite the record of its cycle, votes included.
        require(a.prover == address(0), "Prove : id already used");
        lastRound[msg.sender] = id;

        a.prover = msg.sender;
        a.seedBlock = uint64(block.number);
        a.flipSeqAt = flipSeq;
        a.participantCount = uint16(participants.length);
        a.iterCount = uint16(iterCount);
        a.boundAge = uint8(block.number - boundBlock);
        a.freshMeasurement = freshMeasurement;

        // The reference is already on chain, so the jurors can evaluate at
        // once. The SECaaS verifies, and is notified, only when no jury can be
        // drawn. A jury cycle has no single verifier: its jurors are drawn at
        // the first vote.
        uint256 juryCount = _juryCount(msg.sender);
        if (juryCount == 0) {
            a.verifier = secaas;
            a.state = AttestationState.Open;
            emit AttestationStarted(id, iterCount);
        } else {
            a.juryCount = uint8(juryCount);
            a.state = AttestationState.ReadyForEvaluation;
            emit ReadyForEvaluation(id);
        }
    }

    // A juror gives its verdict. The first vote draws the jury and freezes it,
    // so that later changes to the eligible list do not change who may vote.
    // juryThreshold matching votes close the cycle. As soon as neither verdict
    // can still reach it, the cycle goes to the SECaaS, whose verdict is final:
    // with juryThreshold equal to jurySize, any disagreement goes there.
    function Vote(bytes32 id, bool verdict) external {
        MutualAttestation storage a = attestation[id];
        require(a.prover != address(0), "Vote : unknown cycle");
        require(a.state != AttestationState.Closed, "Vote : cycle closed");
        require(a.verifier != secaas, "Vote : cycle is with the SECaaS");
        require(block.number > a.seedBlock, "Vote : evidence block not sealed yet");
        if (!a.juryDrawn) _freezeJury(a, PreviewJury(id));

        uint256 size = a.jury.length;
        uint256 i = 0;
        while (i < size && a.jury[i].account != msg.sender) i++;
        require(i < size, "Vote : caller is not in the jury");
        Juror storage seat = a.jury[i];
        require(seat.vote == NO_VOTE, "Vote : already voted");
        seat.vote = verdict ? FOR : AGAINST;

        uint256 votesFor = a.votesFor;
        uint256 votesAgainst = a.votesAgainst;
        if (verdict) a.votesFor = uint8(++votesFor);
        else a.votesAgainst = uint8(++votesAgainst);
        emit VoteCast(id, msg.sender, verdict);

        if (votesFor >= juryThreshold || votesAgainst >= juryThreshold) {
            a.verifier = msg.sender; // the juror whose vote closed the cycle, for GetAttestationInfo
            _close(id, votesFor >= juryThreshold);
        } else {
            uint256 left = size - votesFor - votesAgainst;
            if (votesFor + left < juryThreshold && votesAgainst + left < juryThreshold) _toSecaas(id);
        }
    }

    // Anyone may hand a jury cycle to the SECaaS once juryTimeoutBlocks
    // blocks have passed since its evidence without a close.
    function Expire(bytes32 id) external {
        MutualAttestation storage a = attestation[id];
        require(a.prover != address(0), "Expire : unknown cycle");
        require(a.state != AttestationState.Closed, "Expire : cycle closed");
        require(a.verifier != secaas, "Expire : cycle is with the SECaaS");
        require(block.number >= a.seedBlock + juryTimeoutBlocks, "Expire : timeout not reached");
        // The jurors that did not vote lose their eligibility until their next
        // success. Without a vote the jury was never frozen: draw it now, from
        // the agents eligible at the evidence. Past 256 blocks the evidence
        // block hash is gone, and when the draw is unknown no one can be named
        // absent either.
        if (!a.juryDrawn) {
            bytes32 blockHash = blockhash(a.seedBlock);
            if (blockHash != bytes32(0)) {
                (address[] memory jury, bool known) = _cycleJury(a, id, blockHash);
                if (known) _freezeJury(a, jury);
            }
        }
        for (uint256 i = 0; i < a.jury.length; i++) {
            Juror storage seat = a.jury[i];
            if (seat.vote == NO_VOTE) {
                _setEligible(seat.account, false);
                emit JurorAbsent(id, seat.account);
            }
        }
        _toSecaas(id);
    }

    function VerifyBySECaaS(bytes32 id, bool verified) external {
        require(msg.sender == secaas, "Only the SECaaS can resolve attestations");
        MutualAttestation storage a = attestation[id];
        require(a.state == AttestationState.Open, "Not in Open state");
        require(a.verifier == secaas, "SECaaS not elected as verifier");
        _close(id, verified);
    }

    function _freezeJury(MutualAttestation storage a, address[] memory jury) internal {
        for (uint256 i = 0; i < jury.length; i++) a.jury.push(Juror(jury[i], NO_VOTE));
        a.juryDrawn = true;
    }

    function _boundBlock(MutualAttestation storage a) internal view returns (uint256) {
        return a.seedBlock - a.boundAge;
    }

    function _toSecaas(bytes32 id) internal {
        MutualAttestation storage a = attestation[id];
        a.verifier = secaas;
        a.state = AttestationState.Open;
        emit AttestationStarted(id, a.iterCount);
    }

    function _close(bytes32 id, bool verified) internal {
        MutualAttestation storage a = attestation[id];
        a.state = AttestationState.Closed;
        a.result = verified ? AttestationResult.Success : AttestationResult.Failure;
        _setEligible(a.prover, verified);
        // A juror that voted against the verdict, of the jury or of the SECaaS,
        // loses its eligibility until its next success.
        uint8 against = verified ? AGAINST : FOR;
        for (uint256 i = 0; i < a.jury.length; i++) {
            Juror storage seat = a.jury[i];
            if (seat.vote == against) {
                _setEligible(seat.account, false);
                emit JurorDissent(id, seat.account);
            }
        }
        emit AttestationCompleted(id, a.prover, verified);
    }

    // ---------------------------------------------------------------- jury draw

    function _setEligible(address who, bool on) internal {
        Eligibility storage e = eligibility[who];
        uint32 slot = e.slot;
        if (on == (slot != 0)) return;
        uint32 seq = ++flipSeq;
        e.flippedBefore = e.flippedAt;
        e.flippedAt = seq;
        if (on) {
            eligible.push(who);
            e.slot = uint32(eligible.length);
            return;
        }
        address last = eligible[eligible.length - 1];
        eligible[slot - 1] = last;
        eligibility[last].slot = slot;
        eligible.pop();
        e.slot = 0;
    }

    // Number of jurors a cycle of prover gets, 0 when the SECaaS must decide.
    function _juryCount(address prover) internal view returns (uint256) {
        uint256 pool = eligible.length - (eligibility[prover].slot != 0 ? 1 : 0);
        uint256 size = jurySize < pool ? jurySize : pool;
        return size < juryThreshold ? 0 : size;
    }

    // Draws size jurors among the first count participants, keeping those
    // eligible when flipSeq was cutoff. Indexes run over participants, whose
    // order does not change when eligibility does, so the draw does not
    // depend on the changes made since. known is false when a drawn agent
    // changed twice since the cutoff, so its eligibility then is lost, or when
    // size jurors cannot be found, which a removal during the cycle can cause.
    // Expire then still hands the cycle to the SECaaS.
    // ponytail: rejection loop, fine while most agents are eligible; RemoveAgent
    // reorders participants, so a removal during an open cycle changes its draw.
    function _draw(bytes32 seed, address prover, uint256 size, uint32 cutoff, uint256 count)
        internal view returns (address[] memory jury, bool known)
    {
        jury = new address[](size);
        if (participants.length < count) return (jury, false);
        uint256 found = 0;
        for (uint256 k = 0; found < size; k++) {
            if (k >= 64 * count) return (jury, false);
            address candidate = participants[uint256(keccak256(abi.encode(seed, k))) % count];
            if (candidate == prover) continue;
            (bool wasEligible, bool wasKnown) = _wasEligible(candidate, cutoff);
            if (!wasKnown) return (jury, false);
            if (!wasEligible) continue;
            bool taken = false;
            for (uint256 j = 0; j < found; j++) {
                if (jury[j] == candidate) { taken = true; break; }
            }
            if (!taken) jury[found++] = candidate;
        }
        return (jury, true);
    }

    // Whether who was eligible when flipSeq was cutoff. known is false when
    // who changed twice since: its eligibility then is lost.
    function _wasEligible(address who, uint32 cutoff) internal view returns (bool wasEligible, bool wasKnown) {
        Eligibility storage e = eligibility[who];
        bool isEligible = e.slot != 0;
        if (e.flippedAt <= cutoff) return (isEligible, true);
        if (e.flippedBefore <= cutoff) return (!isEligible, true);
        return (false, false);
    }

    function _cycleJury(MutualAttestation storage a, bytes32 id, bytes32 blockHash)
        internal view returns (address[] memory, bool)
    {
        return _draw(keccak256(abi.encode(blockHash, id)), a.prover, a.juryCount, a.flipSeqAt, a.participantCount);
    }

    // The jury of a cycle, seeded by the hash of the block holding its
    // evidence and by its id. Readable from the next block on, so that the
    // prover commits to its evidence before the draw is known.
    function PreviewJury(bytes32 id) public view returns (address[] memory) {
        MutualAttestation storage a = attestation[id];
        require(a.prover != address(0), "PreviewJury : unknown cycle");
        require(block.number > a.seedBlock, "PreviewJury : evidence block not sealed yet");
        bytes32 blockHash = blockhash(a.seedBlock);
        require(blockHash != bytes32(0), "PreviewJury : evidence block too old");
        (address[] memory jury, bool known) = _cycleJury(a, id, blockHash);
        require(known, "PreviewJury : jury cannot be drawn");
        return jury;
    }

    // Draws min(jurySize, eligible agents other than the prover) distinct
    // jurors from seed, from the agents eligible now. Returns an empty jury
    // when fewer than juryThreshold can be drawn. Not the draw of a cycle,
    // which PreviewJury gives: used by the tests.
    function DrawJury(bytes32 seed, address prover) external view returns (address[] memory jury) {
        (jury, ) = _draw(seed, prover, _juryCount(prover), flipSeq, participants.length);
    }

    // ---------------------------------------------------------------- views
    // Access checks use the caller; the address arguments stay for the
    // clients shared with LV and V2. They limit what this contract serves:
    // transaction inputs and storage stay readable from the chain itself.

    function GetAgentInfo(address agentAddress, address) external view returns (string memory, bool) {
        require(agent[msg.sender].registered || msg.sender == secaas, "GetAgentInfo : caller is not registered");
        Agent storage found = agent[agentAddress];
        return (found.name, found.registered);
    }

    function IsRegistered(address callAddress) external view returns (bool) {
        return agent[callAddress].registered;
    }

    function GetEligibleAgents() external view returns (address[] memory) {
        return eligible;
    }

    // The frozen jury of a cycle and each juror's vote (0 no vote, 1 against,
    // 2 for), both empty before the first vote.
    function GetJury(bytes32 id) external view returns (address[] memory jurors, uint8[] memory votes) {
        Juror[] storage jury = attestation[id].jury;
        jurors = new address[](jury.length);
        votes = new uint8[](jury.length);
        for (uint256 i = 0; i < jury.length; i++) {
            jurors[i] = jury[i].account;
            votes[i] = jury[i].vote;
        }
    }

    // The evidence, the anchored reference, K and the block the evidence is
    // bound to. The verifier binds the reference to the hash of that block,
    // sha256(reference || blockhash), and folds it K times: the result equals
    // the evidence of an unmodified agent. Only who may judge the cycle reads
    // it: the SECaaS when it decides, otherwise an agent that can be drawn in
    // its jury (eligible at the evidence, not the prover). The jury itself is
    // known from the next block only, so its candidates may read.
    function GetAttestationSignatures(bytes32 id, address)
        external view returns (bytes32, bytes32, uint32, uint64)
    {
        MutualAttestation storage a = attestation[id];
        require(a.prover != address(0), "Unknown attestation");
        require(a.state == AttestationState.ReadyForEvaluation || a.verifier == secaas,
                "Attestation not ready for evaluation");
        bool allowed;
        if (a.verifier == secaas) {
            allowed = msg.sender == secaas;
        } else if (msg.sender != a.prover) {
            (bool wasEligible, bool wasKnown) = _wasEligible(msg.sender, a.flipSeqAt);
            allowed = wasEligible && wasKnown;
        }
        require(allowed, "GetAttestationSignatures : caller does not judge this cycle");
        return (a.freshMeasurement, referenceOf[a.prover], a.iterCount, uint64(_boundBlock(a)));
    }

    function GetAttestationInfo(bytes32 id) external view returns (address, address, AttestationResult) {
        MutualAttestation storage a = attestation[id];
        require(a.state == AttestationState.Closed, "Attestation process is not closed");
        return (a.prover, a.verifier, a.result);
    }

    function GetAttestationState(bytes32 id) external view returns (AttestationState) {
        return attestation[id].state;
    }

    function IsVerifier(bytes32 id, address callAddress) external view returns (bool) {
        MutualAttestation storage a = attestation[id];
        require(a.state != AttestationState.Closed, "Attestation process is closed or not exists");
        return a.verifier == callAddress;
    }
}
