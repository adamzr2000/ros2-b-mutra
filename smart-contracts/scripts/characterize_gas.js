// Gas of every function of AttestationManagerV3, on each of its paths, by
// number of agents. Runs on Hardhat's in-memory network, which follows the
// Besu genesis (London rules): gas is deterministic, so it equals the gas on
// the chain. Run by experiment_attestation_sc.py; prints one JSON line per
// transaction, prefixed "GAS ".
//
// Only the functions the protocol relies on: those of every round (Prove,
// Vote, VerifyBySECaaS, Expire) on each path, and the onboarding of a robot
// (RegisterAgent, SetReference). Scenarios, on a fresh contract for each
// number of agents N:
//   setup        RegisterAgent, SetReference (admin approvals) per agent
//   bootstrap    one successful round per agent: the SECaaS judges while
//                fewer than juryThreshold other agents are eligible, then juries
//   steady       ROUNDS rounds with every agent eligible
//   failure      the jury votes failure: the prover loses its eligibility
//   dissent      the jurors disagree: the SECaaS judges, the dissenter loses
//                its eligibility
//   expiry       no juror votes: Expire, then the SECaaS; the absent jurors
//                lose their eligibility
// Rounds that give back the eligibility lost above are not recorded.
// Then, on FRACTION_AGENTS agents, steady rounds with only a fraction of them
// eligible (scenario eligibility): the jury draw rejects ineligible agents.
//
// Paths: Prove to_secaas | to_jury; Vote first (draws the jury) | closing |
// to_secaas (a disagreement hands the round over), "_against" for a failure
// vote; VerifyBySECaaS bootstrap | after_dissent | after_expiry; SetReference
// approval | final_approval.
const { ethers, network } = require("hardhat");

const env = (k, d) => (process.env[k] ? process.env[k] : d);
const AGENTS = env("AGENTS", "4,8,16,32,64,128").split(",").map(Number);
const ROUNDS = Number(env("ROUNDS", "20"));
const FAILURES = Number(env("FAILURES", "5"));
const FRACTIONS = env("ELIGIBLE", "1,0.5,0.25").split(",").map(Number);
const FRACTION_AGENTS = Number(env("FRACTION_AGENTS", "64"));
const JURY_SIZE = Number(env("JURY_SIZE", "2"));
const JURY_THRESHOLD = Number(env("JURY_THRESHOLD", "2"));
const JURY_TIMEOUT = Number(env("JURY_TIMEOUT_BLOCKS", "5"));
const FRESHNESS = Number(env("FRESHNESS_BLOCKS", "15"));
const ADMIN_THRESHOLD = 2; // of 3 admins, as deployed by start.sh

const CLOSED = 2n; // AttestationState {Open, ReadyForEvaluation, Closed}
// Calldata as the agents send it: calldata gas depends on the bytes (16 gas
// per non-zero byte, 4 per zero byte). A round id is "attestation" and 16
// hex digits, as the sidecar makes them; references and measurements are
// hashes, random here.
const hash = () => ethers.hexlify(ethers.randomBytes(32));
const newId = () => ethers.encodeBytes32String("attestation" + ethers.hexlify(ethers.randomBytes(8)).slice(2));
const mine = (n = 1) => network.provider.send("hardhat_mine", [ethers.toQuantity(n)]);
const head = () => ethers.provider.getBlockNumber();

async function wallet() {
  const w = ethers.Wallet.createRandom().connect(ethers.provider);
  await network.provider.send("hardhat_setBalance", [w.address, "0x56BC75E2D63100000"]); // 100 ETH
  return w;
}

// One deployment: the contract, its agents, admins and SECaaS.
let c = null;

// The contract events a receipt holds: AttestationStarted hands a round to
// the SECaaS, ReadyForEvaluation to a jury, AttestationCompleted closes it.
const events = (receipt) => receipt.logs.map((l) => {
  try { return c.am.interface.parseLog(l).name; } catch { return null; }
});

const eligibleCount = async () => (c.am ? (await c.am.GetEligibleAgents()).length : 0);

// Sends a transaction, then prints its gas with the path pathOf(receipt)
// tells once it is mined.
// Scenarios run to prepare the others, not recorded.
const UNRECORDED = new Set(["restore", "prepare"]);

async function tx(scenario, fn, send, pathOf) {
  const eligible = await eligibleCount();
  const sent = await send();
  const receipt = await sent.wait();
  if (receipt.status !== 1) throw new Error(`${fn} failed in scenario ${scenario}`);
  // The path is worked out even when not recorded: the callers read it.
  const path = typeof pathOf === "function" ? await pathOf(receipt) : pathOf;
  if (UNRECORDED.has(scenario)) return receipt;
  const logBytes = receipt.logs.reduce((s, l) => s + 32 * l.topics.length + (l.data.length - 2) / 2, 0);
  console.log("GAS " + JSON.stringify({
    agents: c.n, eligible_before: eligible, scenario, function: fn, path,
    gas: Number(receipt.gasUsed), calldata_bytes: (sent.data.length - 2) / 2, log_bytes: logBytes,
  }));
  return receipt;
}

async function deploy(n) {
  const [deployer] = await ethers.getSigners(); // the SECaaS deploys
  c = { n, am: null, secaas: deployer, admins: [], agents: [] };
  for (let i = 0; i < 3; i++) c.admins.push(await wallet());
  for (let i = 0; i < n; i++) c.agents.push(await wallet());
  c.references = new Map(c.agents.map((a) => [a.address, hash()]));
  const factory = await ethers.getContractFactory("AttestationManagerV3", deployer);
  const am = await factory.deploy(c.admins.map((a) => a.address), ADMIN_THRESHOLD, JURY_SIZE, JURY_THRESHOLD,
                                  JURY_TIMEOUT, FRESHNESS);
  await am.waitForDeployment();
  c.am = am;
  for (const a of c.agents) await tx("setup", "RegisterAgent", () => am.connect(a).RegisterAgent("robot"), "register");
  for (const a of c.agents) {
    for (let k = 0; k < ADMIN_THRESHOLD; k++) {
      await tx("setup", "SetReference", () => am.connect(c.admins[k]).SetReference(a.address, c.references.get(a.address)),
               k === ADMIN_THRESHOLD - 1 ? "final_approval" : "approval");
    }
  }
}

const byAddress = (addr) => c.agents.find((a) => a.address === addr);

async function prove(scenario, prover) {
  const id = newId();
  let toSecaas = false;
  await tx(scenario, "Prove", async () => c.am.connect(prover).Prove(id, hash(), 1, await head()), async (receipt) => {
    toSecaas = events(receipt).includes("AttestationStarted");
    return toSecaas ? "to_secaas" : "to_jury";
  });
  return { id, toSecaas };
}

// verdicts[k]: the vote of the k-th juror drawn. Returns the jurors and
// whether the last vote handed the round to the SECaaS.
async function vote(scenario, id, verdicts) {
  await mine(); // the draw reads the hash of the sealed evidence block
  const jury = await c.am.PreviewJury(id);
  let handedOver = false;
  for (const [k, verdict] of verdicts.entries()) {
    await tx(scenario, "Vote", () => c.am.connect(byAddress(jury[k])).Vote(id, verdict), async (receipt) => {
      const names = events(receipt);
      handedOver = names.includes("AttestationStarted");
      const path = names.includes("AttestationCompleted") ? "closing" : handedOver ? "to_secaas"
        : k === 0 ? "first" : "middle";
      return path + (verdict ? "" : "_against");
    });
  }
  return { jury, handedOver };
}

const secaasVerdict = (scenario, id, path, verdict = true) =>
  tx(scenario, "VerifyBySECaaS", () => c.am.connect(c.secaas).VerifyBySECaaS(id, verdict), path);

// A successful round of prover, judged by whoever the contract picks.
async function succeed(scenario, prover) {
  const { id, toSecaas } = await prove(scenario, prover);
  if (toSecaas) await secaasVerdict(scenario, id, "bootstrap");
  else await vote(scenario, id, Array(JURY_THRESHOLD).fill(true));
}

const pick = (list, k) => list[k % list.length];

async function scenariosFor(n) {
  await deploy(n);
  for (const a of c.agents) await succeed("bootstrap", a);
  if ((await eligibleCount()) !== n) throw new Error(`${n} agents, ${await eligibleCount()} eligible`);

  for (let r = 0; r < ROUNDS; r++) await succeed("steady", pick(c.agents, r));

  for (let r = 0; r < FAILURES; r++) {
    const prover = pick(c.agents, r);
    const { id, toSecaas } = await prove("failure", prover);
    if (toSecaas) throw new Error("failure scenario: no jury");
    await vote("failure", id, Array(JURY_THRESHOLD).fill(false));
    if ((await c.am.GetAttestationState(id)) !== CLOSED) throw new Error("failure scenario: round not closed");
    await succeed("restore", prover);
  }

  // Alternating verdicts until neither can reach the threshold: the round
  // goes to the SECaaS, whose success verdict costs the "against" jurors
  // their eligibility.
  const dissent = Array.from({ length: JURY_SIZE }, (_, k) => k % 2 === 0);
  for (let r = 0; r < FAILURES; r++) {
    const { id, toSecaas } = await prove("dissent", pick(c.agents, r));
    if (toSecaas) throw new Error("dissent scenario: no jury");
    const { jury, handedOver } = await vote("dissent", id, dissent);
    if (!handedOver) throw new Error("dissent scenario: not handed to the SECaaS");
    await secaasVerdict("dissent", id, "after_dissent");
    for (const [k, verdict] of dissent.entries()) if (!verdict) await succeed("restore", byAddress(jury[k]));
  }

  for (let r = 0; r < FAILURES; r++) {
    const prover = pick(c.agents, r);
    const { id, toSecaas } = await prove("expiry", prover);
    if (toSecaas) throw new Error("expiry scenario: no jury");
    await mine(JURY_TIMEOUT);
    const jury = await c.am.PreviewJury(id);
    await tx("expiry", "Expire", () => c.am.connect(prover).Expire(id), "expire");
    await secaasVerdict("expiry", id, "after_expiry");
    for (const j of jury) await succeed("restore", byAddress(j));
  }

}

async function eligibilitySweep() {
  await deploy(FRACTION_AGENTS);
  for (const fraction of FRACTIONS) {
    const subset = c.agents.slice(0, Math.max(JURY_THRESHOLD + 1, Math.round(fraction * c.n)));
    for (const a of subset) await succeed("prepare", a);
    for (let r = 0; r < ROUNDS; r++) await succeed("eligibility", pick(subset, r));
    for (let k = 0; k < ADMIN_THRESHOLD; k++) await c.am.connect(c.admins[k]).ResetChain();
  }
}

async function main() {
  console.log("INFO " + JSON.stringify({
    hardfork: network.config.hardfork, agents: AGENTS, rounds: ROUNDS, failures: FAILURES,
    eligible_fractions: FRACTIONS, fraction_agents: FRACTION_AGENTS, jury_size: JURY_SIZE,
    jury_threshold: JURY_THRESHOLD, jury_timeout_blocks: JURY_TIMEOUT, freshness_blocks: FRESHNESS,
    admins: 3, admin_threshold: ADMIN_THRESHOLD,
  }));
  for (const n of AGENTS) await scenariosFor(n);
  await eligibilitySweep();
  console.log("DONE");
}

main().catch((e) => {
  console.error(e);
  process.exitCode = 1;
});
