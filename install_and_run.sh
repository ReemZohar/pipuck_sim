#!/bin/bash
set -e

# Run install.sh from the repository root
echo "Running install.sh..."
./install.sh

# Enter the pipuck_ws workspace
echo "Entering pipuck_ws..."
cd pipuck_ws

# Compile the workspace
echo "Running compile_ws.sh..."
./compile_ws.sh

# Run the simulation using ARGoS
echo "Running simulation..."
argos3 -c experiments/phybot_test.argos
