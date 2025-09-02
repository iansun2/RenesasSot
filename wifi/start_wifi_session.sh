#!/bin/bash

TMUX_SESSION_NAME="wifi"
RUN_SCRIPT="wifi_conn.sh"

cd "$(dirname "$0")"
tmux new-session -d -s ${TMUX_SESSION_NAME} 
tmux send-keys -t ${TMUX_SESSION_NAME}:0 "./${RUN_SCRIPT}" C-m

