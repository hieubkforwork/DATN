#!/bin/bash

set -e

YOCTO_DIR=yocto
POKY_REPO=https://git.yoctoproject.org/poky
BRANCH=kirkstone

echo "Cloning Poky..."

if [ ! -d "$YOCTO_DIR" ]; then
    git clone -b $BRANCH $POKY_REPO $YOCTO_DIR
else
    echo "Poky already exists"
fi

echo "Entering Yocto directory..."
cd $YOCTO_DIR

echo "Setup build environment..."
source oe-init-build-env build

echo "Yocto environment ready"
