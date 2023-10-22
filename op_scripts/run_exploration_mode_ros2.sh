#############################
# Map exploration Mode
# Load the map specified with "FREE_MAP_NAME" and attach the ego vehicle agent to the only vehile in that map 
# No scenario required, the ego vehicle should explore the scene 
#############################

export SIMULATOR_LOCAL_HOST="localhost"
# export SIMULATOR_LOCAL_HOST="192.168.11.5"
export SIMULATOR_PORT="2000"
export TEAM_AGENT=${LEADERBOARD_ROOT}/op_bridge/op_ros2_agent.py
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}
export AGENT_FRAME_RATE="60"
# Autonomous actor default role_name
export AGENT_ROLE_NAME="hero"

# modes are 
#   * "leaderboard" : when runner the leaderboard (route based) scenario collections 
#   * "srunner" : when scenario is loaded using scenario runner, and only agent is attached
#   * "free" : when loading empty map only , either carla town or any OpenDRIVE map 
export OP_BRIDGE_MODE="free" 

# CARLA town name or custom OpenDRIVE absolute path, when BRIDGE_MODE is free 
export FREE_MAP_NAME="SUSTC_ParkingLot" 
# export FREE_MAP_NAME="Town04" 

# Spawn point for the autonomous agent, when BRIDGE_MODE is free 
# "x,y,z,roll,pitch,yaw"
# Empty string means random starting position
#export FREE_AGENT_POSE="175.4,195.14,0,0,0,180" 
# export FREE_AGENT_POSE="-29.1,11.3,-4,0,0,90" #从a到b
# export FREE_AGENT_POSE="-18,27,-4,0,0,-45" # 斜向倒车入库
# export FREE_AGENT_POSE="-17.5,26.5,-4,0,0,0" # 横向倒车
export FREE_AGENT_POSE="-8.2,28.2,-4,0,0,0" # 用来测试离线纯视觉BEV的自车起点

#gnome-terminal -- bash -c roscore
python3 ${LEADERBOARD_ROOT}/op_bridge/op_bridge_ros2.py

