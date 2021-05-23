#!/usr/bin/env bash

set -e

echo "building containers for noetic..."

docker build -t ros:noetic-ros-base-l4t-r32.5.1 -f Dockerfile.ros .
docker build -t rosforce:noetic-jetson --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" --build-arg ssh_pub_key="$(cat ~/.ssh/id_rsa.pub)" -f Dockerfile .