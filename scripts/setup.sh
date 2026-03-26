#!/bin/bash

set -e

echo "===== STEP: Build Docker image ====="

docker build -t yocto-builder .

echo "===== DONE ====="
echo "Docker image 'yocto-builder' is ready"
