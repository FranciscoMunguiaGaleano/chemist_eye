#!/bin/bash

HOST="chemisteyeirtwo@192.168.1.105"
SCRIPTS_PATH="~/chemist_eye_ir/src"
PAUSE_DURATION=5  # Set the pause duration in seconds

echo "Waiting for $PAUSE_DURATION seconds before attempting to connect..."
sleep $PAUSE_DURATION

# Run all commands in a single SSH session
ssh $HOST << EOF
cd $SCRIPTS_PATH || exit 1
python stream_ir.py || exit 1
EOF

# Check if the SSH session was successful
if [ $? -ne 0 ]; then
    echo "[ERROR]: Failed to execute script on remote host $HOST"
    exit 1
fi

echo "Script executed successfully."
