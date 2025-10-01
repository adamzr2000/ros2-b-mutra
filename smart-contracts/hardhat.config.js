require("@nomicfoundation/hardhat-toolbox");
require("dotenv").config();

const { PRIVATE_KEY, RPC_URL, CHAIN_ID } = process.env;

module.exports = {
  solidity: "0.8.28",
  networks: {
    besu: {
      url: RPC_URL || "http://localhost:8545",
      chainId: CHAIN_ID ? parseInt(CHAIN_ID) : 1337,
      accounts: PRIVATE_KEY ? [PRIVATE_KEY] : [],
      gasPrice: 0, // Besu/QBFT devnets typically accept 0-gas legacy txs
    },
  },
};
