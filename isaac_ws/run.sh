#!/bin/bash

# Define default values
isaac_sim_path="$HOME/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh"
python_file="quadruped_robot.py"

# Check if arguments were provided, and if not, use default values
if [ $# -eq 0 ]; then
    arg1="$isaac_sim_path"
    arg2="$python_file"
else
    # Use the provided arguments
    arg1="$1"
    arg2="$2"
fi

# Print the values of the arguments
echo "Argument 1: $arg1"
echo "Argument 2: $arg2"

# Run the python file
"$arg1" "$arg2"