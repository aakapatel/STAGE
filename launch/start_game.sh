#!/bin/bash


SESSION=acl_main

# Split Panes
tmux -2 new-session -d -s $SESSION

tmux split-window -h
tmux select-pane -t 0
tmux split-window -h
tmux select-pane -t 1
tmux split-window -v 
tmux select-pane -t 2
tmux split-window -v 
tmux select-pane -t 3
tmux split-window -v 
tmux select-pane -t 4
tmux split-window -v 
tmux select-pane -t 5
tmux split-window -v 
tmux select-pane -t 6
tmux split-window -v 
tmux select-pane -t 7
tmux split-window -v 
tmux select-pane -t 8
tmux split-window -v
tmux select-pane -t 9
tmux split-window -v 
tmux select-pane -t 10
tmux split-window -v 
tmux select-pane -t 11
tmux split-window -v 
tmux select-pane -t 12
tmux split-window -h
tmux select-pane -t 13
tmux split-window -v  

# tmux send-keys -t 12 "sleep 1; roscore" C-m

tmux send-keys -t 1 'sleep 2; cd catkin_workspaces/multiagent_ws/; sr; ' C-m
tmux send-keys -t 1 'sleep 1; roslaunch rotors_gazebo mav_hovering_example.launch ' C-m

tmux send-keys -t 2 'sleep 2; cd catkin_workspaces/multiagent_ws/; sr; ' C-m
tmux send-keys -t 2 'sleep 3; cd src/python; ' C-m
tmux send-keys -t 2 'sleep 1; python3 hummingbird_multimav.py ' C-m

tmux send-keys -t 3 'sleep 2; cd catkin_workspaces/multiagent_ws/; sr; ' C-m
tmux send-keys -t 3 'sleep 3; cd src/python; ' C-m
tmux send-keys -t 3 'sleep 1; python3 hummingbird_PF.py ' C-m

tmux send-keys -t 4 'sleep 5; cd catkin_workspaces/exploration_ws/; sr; ' C-m
tmux send-keys -t 4 'sleep 6; roslaunch acl_exploration acl_exploration.launch ' C-m

tmux send-keys -t 5 'sleep 5; cd catkin_workspaces/exploration_ws/; sr; ' C-m
tmux send-keys -t 5 'sleep 6; roslaunch acl_exploration main.launch ' C-m

tmux send-keys -t 6 'sleep 5; cd catkin_workspaces/exploration_ws/; sr; ' C-m
tmux send-keys -t 6 'sleep 1; cd src/acl_exploration; python3 mission.py ' C-m

tmux send-keys -t 7 "sleep 2; cd catkin_workspaces/multiagent_ws; sr; " C-m
tmux send-keys -t 7 'sleep 3; rostopic pub /hummingbird/set_start std_msgs/String "data: ''" -1 ' C-m

tmux send-keys -t 8 'sleep 2; cd catkin_workspaces/exploration_ws/; sr; ' C-m 
tmux send-keys -t 8 'sleep 6; rostopic pub /explore std_msgs/String "data: ''" -1 ' C-m

tmux send-keys -t 9 'sleep 3; cd catkin_workspaces/exploration_ws/; sr; ' C-m 
tmux send-keys -t 9 'rostopic pub /return_home std_msgs/String "data: ''" -1 ' 

tmux send-keys -t 10 'sleep 1; cd catkin_workspaces/exploration_ws/; sr; ' C-m 
tmux send-keys -t 10 'sleep 1; cd src/rosbags ' C-m 
tmux send-keys -t 10 'sleep 1; ./rosbag_alc.sh' C-m

tmux send-keys -t 13 'sleep 5; cd catkin_workspaces/exploration_ws/; sr; ' C-m 

tmux -2 attach-session -t $SESSION
