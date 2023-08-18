#!/bin/bash
devel_file=$(pwd)/../../devel/setup.bash
session="jackal"
tmux new-session -d -s $session
window=0
tmux rename-window -t $session:$window 'launch' \; split-window -h
tmux send-keys -t $session.0 'ssh administrator@jackal' ENTER
tmux send-keys -t $session.1 'ssh administrator@jackal' ENTER
sleep 2
tmux send-keys -t $session.0 'clearpath' ENTER
tmux send-keys -t $session.1 'clearpath' ENTER
sleep 1
tmux send-keys -t $session.0 'source ~/.bashrc' ENTER
tmux send-keys -t $session.1 'source ~/.bashrc' ENTER
tmux send-keys -t $session.0 'roslaunch kinova_bringup kinova_robot.launch' ENTER
sleep 5
tmux send-keys -t $session.1 'rosrun kinova_demo joints_action_client.py -v j2n6s300 radian -- 0 2.9 1.3 4.2 1.4 0' ENTER
tmux attach -t jackal
