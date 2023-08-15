tmux send-keys -t mobiman.0 'roslaunch mobiman_simulation gazebo.launch' ENTER
sleep 5
tmux send-keys -t mobiman.1 'roslaunch mobiman_simulation ocs2_target.launch' ENTER
sleep 2
tmux send-keys -t mobiman.2 'roslaunch mobiman_simulation ocs2_m4.launch' ENTER
sleep 1