#!/bin/bash

SESSION="ros2_echo"

tmux new-session -d -s $SESSION

# First pane (default)
tmux send-keys -t $SESSION "ros2 topic echo /deadwheeel_ticks" C-m

# Split horizontally
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "ros2 topic echo /odom_deadwheels" C-m

# Split vertically (on first pane)
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "ros2 topic echo /odometry/local" C-m

tmux select-pane -t 0
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "ros2 topic echo /camera/global_pose" C-m

# Optional: even layout
tmux select-layout -t $SESSION tiled

# Attach to session
tmux attach -t $SESSION