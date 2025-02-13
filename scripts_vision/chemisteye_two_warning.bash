#!/bin/bash

HOST="chemisteyetwo@192.168.1.102"
SCRIPTS_PATH="~/chemist_eye/src"

# Run all commands in a single SSH session
ssh $HOST << EOF
cd $SCRIPTS_PATH || exit 1
ls || exit 1
bash speech.bash || exit 1
EOF

# Check if the SSH session was successful
if [ $? -ne 0 ]; then
    echo "[ERROR]: Failed to execute script on remote host $HOST"
    exit 1
fi

echo "Script executed successfully."
