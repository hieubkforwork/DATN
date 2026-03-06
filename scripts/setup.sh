#!/bin/bash

set -e

echo "===== STEP 1: Clone Poky ====="

if [ ! -d "../poky" ]; then
    git clone https://git.yoctoproject.org/git/poky ../poky
fi

cd ../poky
git checkout kirkstone
cd ..

echo "===== STEP 2: Build Docker image ====="

docker build -t yocto-builder ..

echo "===== DONE ====="
echo "Docker image 'yocto-builder' is ready"
