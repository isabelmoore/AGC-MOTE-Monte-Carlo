#!/bin/bash
SESSION="ros_sim"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# 1. Start a new detached tmux session
tmux new-session -d -s $SESSION -n 'Processing'

# 2. In the first window, Run the Filter & Bag Play
tmux send-keys -t $SESSION 'docker-compose run --rm mote_ros /bin/bash -c "source devel/setup.bash && roslaunch mote_ros run_both_nodes.launch"' C-m

# 3. Create a second window for RViz
tmux new-window -t $SESSION -n 'RViz'
tmux send-keys -t $SESSION:1 'docker-compose run --rm mote_ros /bin/bash -c "source devel/setup.bash && roslaunch mote_ros rviz.launch"' C-m

# 4. Attach to the session (defaulting to Processing window)
tmux select-window -t $SESSION:0
tmux attach-session -t $SESSION
