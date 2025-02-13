#!/bin/bash

HOST="chemisteyetwo@192.168.1.102"
SCRIPTS_PATH="~/chemist_eye/src"
PAUSE_DURATION=5  # Set the pause duration in seconds

echo "Waiting for $PAUSE_DURATION seconds before attempting to connect..."
sleep $PAUSE_DURATION

# Run all commands in a single SSH session
ssh $HOST << EOF
source /home/chemisteyetwo/miniconda3/etc/profile.d/conda.sh
conda activate chemist_eye
cd $SCRIPTS_PATH || exit 1
bash starting_announcement.bash || exit 1
python streamer.py || exit 1
EOF

# Check if the SSH session was successful
if [ $? -ne 0 ]; then
    echo "[ERROR]: Failed to execute script on remote host $HOST"
    exit 1
fi

echo "Script executed successfully."
