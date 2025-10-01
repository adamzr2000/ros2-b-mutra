#!/usr/bin/env bash

docker run -it \
  --rm \
  --name quorum-dev \
  -u "$(id -u):$(id -g)" \
  -v $(pwd)/:/workspace:rw \
  quorum-dev \
  npx quorum-dev-quickstart