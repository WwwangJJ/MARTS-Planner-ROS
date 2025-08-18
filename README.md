# MARTS-Planner
Code for paper "Safe and Agile Transportation of Cable-Suspended Payload via Multiple Aerial Robots"

## Applications

### Example 1: Replan Mode
This mode is used for re-planning, just as described in the paper, the planning time required for each re-planning is within 100ms, and the planning success rate can reach 100%, under the premise of reasonable parameter selection. For installation, the following terminal commands are helpful.
    
    sudo apt update
    sudo apt install cpufrequtils
    sudo apt install libompl-dev
    
    mkdir transport-multiple; cd transport-multiple; mkdir src; cd src
    git clone https://github.com/ZJU-FAST-Lab/MARTS-Planner.git
    cd ..
    
    catkin_make -DCATKIN_WHITELIST_PACKAGES="quadrotor_msgs"
    catkin_make -DCATKIN_WHITELIST_PACKAGES=""
    roslaunch gcopter Astar_planning_RM.launch

After conduct the command, you will see the window for rviz. Please follow the gif below for trajectory planning in a random map.
    
<p align = "center">
    <img src="Figs/replan.gif"/>
</p>
