#!/bin/bash

# Function to handle termination signal
function cleanup() {
    echo "Termination signal received. Cleaning up..."
    # Kill the 'player' process
    if [ -n "$player_pid" ]; then
        kill -TERM "$player_pid"
        wait "$player_pid"
    fi
    # Exit the script
    exit 0
}

# Register the cleanup function to catch termination signals
trap "cleanup" SIGINT SIGTERM

# Start the first process in the background
player player_simple.cfg &

# Store the process ID (PID) of the 'player' process
player_pid=$!

# Wait for a moment to ensure the 'player' process has started
sleep 2

# Start the second process in a separate directory
cd build
./client

# If the 'client' process is terminated, clean up by killing the 'player' process
cleanup
