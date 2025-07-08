#!/bin/bash

docker run -it \
  --rm \
  --name geth-net-configurator \
  -u "$(id -u):$(id -g)" \
  -v $(pwd)/:/src:rw \
  geth-node:poa \
  ./local_geth_poa_net_setup.sh