#!/bin/bash

echo 'Deleting DLT network'

docker compose down

sudo ./clean_nodes_db.sh

