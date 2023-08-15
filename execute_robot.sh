#!/bin/bash
devel_file=$(pwd)/../../devel/setup.bash
session="mobiman"
tmux new-session -d -s $session
window=0
tmux rename-window -t $session:$window 'launch' \; split-window -h \; split-window -h \; split-window -v \; split-window -v
tmux send-keys -t mobiman.0 'source ../../devel/setup.bash' ENTER
tmux send-keys -t mobiman.1 'source ../../devel/setup.bash' ENTER
tmux send-keys -t mobiman.2 'source ../../devel/setup.bash' ENTER
tmux send-keys -t mobiman.3 'source ../../devel/setup.bash' ENTER
tmux send-keys -t mobiman.4 'source ../../devel/setup.bash' ENTER
tmux send-keys -t mobiman.3 'roslaunch mocap_optitrack mocap.launch &' ENTER
# tmux send-keys -t mobiman.4 'rosrun mobiman_simulation jackal_jaco_transform.py' ENTER
tmux send-keys -t mobiman.3 './ex_mobiman.sh' ENTER
tmux attach -t mobiman

