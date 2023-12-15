import os
import subprocess
import socketio

taskId = os.getenv('TASK_ID')
token = os.getenv('TOKEN')
mode = os.getenv('MODE')
model = os.getenv('MODEL')
planner = os.getenv('PLANNER')
map = os.getenv('MAP')
scenario = os.getenv('SCENARIO')

# TODO: validate these values using the spec files once they are aviailable

if(mode == 'evaluate'):
    subprocess.run(f"source ./devel/setup.sh && roslaunch arena_bringup start_arena.launch visualization:=none record_data:=true model:={model} local_planner:={planner} map_file:={map} scenario_file:={scenario} task_mode:=scenario", shell=True)

elif(mode == 'plot'):
    subprocess.run(f"source ./devel/setup.sh && python3 /root/src/arena-evaluation/create_plots.py plotting_data.yaml", shell=True)

elif(mode == 'train'):
    subprocess.run(f"source ./devel/setup.sh && roslaunch arena_bringup start_training.launch model:={model} map_folder_name=map && python3 src/arena-rosnav/training/scripts/train_agent.py --agent AGENT_22", shell=True)