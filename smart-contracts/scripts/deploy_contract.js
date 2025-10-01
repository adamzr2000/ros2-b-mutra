const { ethers } = require("hardhat");
const fs = require("fs");
const path = require("path");

async function main() {
  const [deployer] = await ethers.getSigners();
  console.log("Deployer:", deployer.address);

  // Legacy-style 0 gasPrice works well on Besu devnets
  const MasMutualAttestation = await ethers.getContractFactory("MasMutualAttestation");
  const contract = await MasMutualAttestation.deploy({ gasPrice: 0 });
  await contract.waitForDeployment();

  const addr = await contract.getAddress();
  console.log("MasMutualAttestation deployed at:", addr);

  // Save address with <networkName>-<contractName>.json
  const networkName = hre.network.name;
  const outDir = path.join(__dirname, "..", "deployments");
  fs.mkdirSync(outDir, { recursive: true });
  const outfile = path.join(outDir, `${networkName}-MasMutualAttestation.json`);
  fs.writeFileSync(
    outfile,
    JSON.stringify(
      {
        contract: "MasMutualAttestation",
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