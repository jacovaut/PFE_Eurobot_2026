#!/bin/bash

source ./install/setup.bash

SESSION="ros2_echo"

# CMD1="ros2 topic echo /deadwheel_ticks"
CMD1="ros2 topic echo /odom_deadwheels --field pose.pose.position"
CMD2="ros2 topic echo /odom_deadwheels --field twist.twist"
CMD3="ros2 topic echo /odometry/local --field pose.pose.position"
CMD4="ros2 topic echo /odometry/local --field twist.twist"
# CMD4="ros2 topic echo /camera/global_pose"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION

# First pane (default)
tmux send-keys -t $SESSION "$CMD1" C-m

# Split horizontally
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "$CMD2" C-m

# Split vertically (on first pane)
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$CMD3" C-m

tmux select-pane -t 0
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "$CMD4" C-m

# Optional: even layout
tmux select-layout -t $SESSION tiled

# Attach to session
exec tmux attach -t $SESSION
