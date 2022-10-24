#!/bin/bash

# Runs a sequence of a dataset.
# Usage: ./run.sh $1 $2 $3 $4
# $1: Path to HyperSLAM executable.
# $2: Path to HyperSLAM settings.
# $3: Path to source file (e.g. rosbag).
# $4: Path to output directory.

# Launch ROS.
printf "\n### ROS >>>\n\n"
roscore &
sleep 2

# Launch HyperSLAM.
printf "\n### HyperSLAM >>>\n\n"
$1 "$2" "$4" &
sleep 2

# Play sequence.
printf "\n### Play >>>\n\n"
rosbag play "$3" --quiet

# Send SIGUSR1 to HyperSLAM and wait.
printf "\n### Waiting ###\n\n"
pkill -SIGUSR1 HyperSLAM
wait %2

# Kill roscore.
printf "\n<<< ROS ###\n\n"
pkill roscore
wait %1
exit 0
