const { ethers } = require("hardhat");
const fs = require("fs");
const path = require("path");

const CONTRACT_NAME = process.env.CONTRACT_NAME || "AttestationManagerLV";

async function main() {
  const [deployer] = await ethers.getSigners();
  console.log("Deployer:", deployer.address);

  const factory = await ethers.getContractFactory(CONTRACT_NAME);

  // LV takes no constructor arguments; V2 and V3 take their admins (and V3 its jury).
  let contract;
  if (CONTRACT_NAME === "AttestationManagerV2" || CONTRACT_NAME === "AttestationManagerV3") {
    // Admin keys that may write the on-chain references. Never the deployer (SECaaS).
    const { ADMINS, ADMIN_THRESHOLD } = process.env;
    if (!ADMINS || !ADMIN_THRESHOLD) {
      throw new Error(`${CONTRACT_NAME} requires ADMINS (comma-separated addresses) and ADMIN_THRESHOLD`);
    }
    console.log(`Admins     : ${ADMINS} (threshold ${ADMIN_THRESHOLD})`);
    const args = [ADMINS.split(","), parseInt(ADMIN_THRESHOLD)];
    if (CONTRACT_NAME === "AttestationManagerV3") {
      const { JURY_SIZE, JURY_THRESHOLD, JURY_TIMEOUT_BLOCKS, FRESHNESS_BLOCKS } = process.env;
      if (!JURY_SIZE || !JURY_THRESHOLD || !JURY_TIMEOUT_BLOCKS || !FRESHNESS_BLOCKS) {
        throw new Error(`${CONTRACT_NAME} requires JURY_SIZE, JURY_THRESHOLD, JURY_TIMEOUT_BLOCKS and FRESHNESS_BLOCKS`);
      }
      console.log(`Jury       : ${JURY_THRESHOLD} of ${JURY_SIZE}, timeout ${JURY_TIMEOUT_BLOCKS} blocks`);
      console.log(`Freshness  : evidence bound to a block at most ${FRESHNESS_BLOCKS} blocks old`);
      args.push(parseInt(JURY_SIZE), parseInt(JURY_THRESHOLD), parseInt(JURY_TIMEOUT_BLOCKS), parseInt(FRESHNESS_BLOCKS));
    }
    contract = await factory.deploy(...args, { gasPrice: 0 });
  } else {
    contract = await factory.deploy({ gasPrice: 0 });
  }
  await contract.waitForDeployment();

  const addr = await contract.getAddress();
  console.log(`${CONTRACT_NAME} deployed at:`, addr);

  // Save address with <networkName>-<contractName>.json
  const networkName = hre.network.name;
  const outDir = path.join(__dirname, "..", "deployments");
  fs.mkdirSync(outDir, { recursive: true });
  const outfile = path.join(outDir, `${networkName}-${CONTRACT_NAME}.json`);
  fs.writeFileSync(
    outfile,
    JSON.stringify(
      {
        contract: CONTRACT_NAME,
        address: addr,
        network: networkName,
      },
      null,
      2
    )
  );
  console.log(`Saved to ${outfile}`);
}

main().catch((e) => {
  console.error(e);
  process.exitCode = 1;
});
