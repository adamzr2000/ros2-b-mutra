module.exports = {
  networks: {
    development: {
      host: "127.0.0.1", 
      port: 7545,
      network_id: "*"
    },

    geth_network_ws: {
      host: process.env.NODE_IP,
      port: process.env.PORT,
      network_id: process.env.NETWORK_ID,
      websockets: true
    },
    geth_network_http: {
      host: process.env.NODE_IP,
      port: process.env.PORT,
      network_id: process.env.NETWORK_ID,
      websockets: false
    },
  },
  compilers: {
    solc: {
      version: "0.8.0",   
    }
  },
};