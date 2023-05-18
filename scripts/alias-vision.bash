
SSH_EMBEDDED="ssh -tt embedded@10.24.4.100"
SSH_VISION=""
SSH_CONTROL="ssh -tt centauro@10.24.4.102"

NUM=0

CMD="ecat"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_EMBEDDED ecat_master"

CMD="xbot"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_EMBEDDED xbot2-core --hw ec_ub_imp"

CMD="fans"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_EMBEDDED \"source .bashrc; /home/embedded/xbot2_ws/src/ec_xbot2_client/scripts/ecat_fans_on.py\""

CMD="rviz"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="source ~/eurobin_ws/setup.bash; rviz -d ~/eurobin_ws/ros_src/eurobin_grasp_box/config/main.rviz"

CMD="base_estimation"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_CONTROL mon launch eurobin_grasp_box centauro_base_estimation.launch"

CMD="action_server"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_CONTROL rosrun centauro_parking action_server.py"

CMD="head_camera"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_VISION \"source eurobin_ws/setup.bash; mon launch centauro_sensors bringup_head_camera.launch\""

CMD="velodyne"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_VISION mon launch hhcm_perception camera_launcher.launch use_velodyne:=true"

CMD="aruco"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_VISION \"source eurobin_ws/setup.bash; mon launch eurobin_grasp_box bringup_robot.launch\""

CMD="plan"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_VISION \"source eurobin_ws/setup.bash; rosrun eurobin_grasp_box planner_main.py\""

CMD="grasp"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_VISION \"source eurobin_ws/setup.bash; rosrun eurobin_grasp_box grasp_the_box_adm_ctrl.py\""

CMD="lift"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_VISION \"source eurobin_ws/setup.bash; rosrun eurobin_grasp_box lift_the_box.py\""

CMD="release"
NUM=$((NUM+1))
alias eurobin-$NUM-$CMD="$SSH_VISION \"source eurobin_ws/setup.bash; rosrun eurobin_grasp_box release_the_box.py\""

CMD="relax_joints"
alias eurobin-$CMD="$SSH_VISION \"source eurobin_ws/setup.bash; rosrun eurobin_grasp_box relax_joints.py\""

CMD="homing_legs"
alias eurobin-$CMD="$SSH_CONTROL rosrun centauro_parking parking.py --action homing_legs"


function eurobin_plan_grasp_home() {

    echo "ENTER to plan"
    read UNUSED
    eurobin-10-plan 
    
    echo "ENTER to grasp"
    read UNUSED
    eurobin-11-grasp
    
    # echo "ENTER to home"
    # read UNUSED
    # eurobin-11-homing_legs
    
    echo "ENTER to lift"
    read UNUSED
    eurobin-12-lift

    echo "ENTER to release"
    read UNUSED
    eurobin-13-release

}


