#!/bin/bash
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (use sudo)"
    exit 1
fi
./pmd_test /dev/ttyUSB0 0.1
docker compose -f compose-scripts/docker-compose.yaml up -d
