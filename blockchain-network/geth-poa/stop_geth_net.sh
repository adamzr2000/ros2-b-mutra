#!/bin/bash

echo 'Deleting geth network'

docker compose down

./clean_nodes_db.sh