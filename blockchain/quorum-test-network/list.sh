#!/bin/bash -u
# Lists the endpoints of the running blockchain stack.

NO_LOCK_REQUIRED=false

. ./.env
. ./.common.sh

HOST=${DOCKER_PORT_2375_TCP_ADDR:-"localhost"}
running() { [[ -n "$(docker compose --profile dashboards ps -q "$1" 2>/dev/null)" ]]; }

echo "----------------------------------"
echo "Endpoints"
echo "----------------------------------"
echo "JSON-RPC HTTP (validator1..4) : http://${HOST}:21001 .. 21004"
if running explorer; then
  echo "Block explorer                : http://${HOST}:25000/explorer/nodes"
  echo "Prometheus                    : http://${HOST}:9090"
  echo "Grafana                       : http://${HOST}:3000/d/XE4V0WGZz/besu-overview"
else
  echo "Dashboards                    : off (./start.sh --dashboards)"
fi
