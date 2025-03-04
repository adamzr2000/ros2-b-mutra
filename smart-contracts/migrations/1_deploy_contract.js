const MasMutualAttestation = artifacts.require('MasMutualAttestation.sol');

module.exports = function (deployer) {
  deployer.deploy(MasMutualAttestation);
};