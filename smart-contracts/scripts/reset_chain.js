// Approves ResetChain in AttestationManagerV2 or V3 (CONTRACT_NAME) as the
// admin PRIVATE_KEY. Run once per admin key: the reset applies once enough
// admins approved it (ChainReset).
const { ethers } = require("hardhat");
const fs = require("fs");
const path = require("path");

async function main() {
  const { CONTRACT_NAME } = process.env;
  if (!CONTRACT_NAME) throw new Error("CONTRACT_NAME is required");

  const deployment = path.join(__dirname, "..", "deployments", `${hre.network.name}-${CONTRACT_NAME}.json`);
  const { address } = JSON.parse(fs.readFileSync(deployment));
  const [admin] = await ethers.getSigners();
  const am = await ethers.getContractAt(CONTRACT_NAME, address, admin);

  const receipt = await (await am.ResetChain({ gasPrice: 0 })).wait();
  const events = receipt.logs.map((log) => am.interface.parseLog(log)).filter(Boolean);
  const approved = events.find((e) => e.name === "ResetApproved");
  const reset = events.some((e) => e.name === "ChainReset");
  console.log(`Admin ${admin.address} approved reset ${approved.args.version}: ${reset ? "chain reset" : "more approvals needed"}`);
}

main().catch((e) => {
  console.error(e);
  process.exitCode = 1;
});
