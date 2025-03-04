#!/bin/bash

docker run -it \
  --rm \
  --name dlt-network-configurator \
  -v $(pwd)/:/dlt-network \
  dlt-node:geth-pow \
  ./local_dlt_network_setup.sh