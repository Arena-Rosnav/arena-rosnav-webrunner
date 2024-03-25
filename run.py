import os
import subprocess
import socketio
import json
import random
from fileCreator import FileCreator

base_path = "~/arena_ws"
taskId = os.getenv('taskId')
tasktype = os.getenv('type')
url = os.getenv('url')
files_recieved = False

def startTask(data):
    
    dataObj = json.load(data)
    msg_taskId = dataObj.get('taskId')
    
    if taskId != msg_taskId:
        return 
    
    print('Recieved files! Starting file creation')
    f = FileCreator(base_path)

    if(tasktype == 'training'):    
        f.createRobotFiles(data)
        f.createWorldFile(data)
        f.createHyperparamsFile(data)
    elif(tasktype == 'evaluation'):
        f.createRobotFiles(data)
        f.createWorldFile(data)
        f.createScenarioFile(data)
    elif(tasktype == 'benchmark'):
        f.createRobotFiles(data)
        f.createWorldFile(data)
        f.createScenarioFile(data)
    else:
        f.createPlotFile(data)

    print('Files created. Starting Arena-Rosnav')
    files_recieved = True

if not (taskId or tasktype or url):
    print('Environment variables are not set. Exiting...')
    exit

sio = socketio.Client()
sio.connect(url=url,transports="websocket")
sio.on('configTransmission', startTask)
sio.emit('configRequest', {'taskId', taskId})

print('Waiting for config files')
while True:
    if files_recieved:
        break

if(tasktype == 'training'):

    model = os.getenv('model')    
    subprocess.run(f"source ./devel/setup.sh && roslaunch arena_bringup start_training.launch model:={model} map_folder_name=map_empty && python3 {base_path}/src/arena/arena-rosnav/training/scripts/train_agent.py --agent AGENT_22", shell=True)
    response = {
        "taskId": taskId,
    }

elif(tasktype == 'evaluation'):
    
    model = os.getenv('model')
    planner = os.getenv('planner')
    subprocess.run(f"source ./devel/setup.sh && roslaunch arena_bringup start_arena.launch tm_robots:=scenario tm_obstacles:=scenario model:={model} local_planner:={planner} map_file:=map_custom record_data:=true", shell=True)

    f = FileCreator(base_path)
    items = os.listdir(f'{base_path}/src/arena/evaluation/arena-evaluation/data') 
    parentFolder = items[0]
    zip64 = f.createBase64Zip(f"{base_path}/src/arena/evaluation/arena-evaluation/data/{parentFolder}")
    response = {
        "taskId": taskId,
        "data": zip64,
    }

elif(tasktype == 'benchmark'):

    model = os.getenv('model')
    planner = os.getenv('planner')
    subprocess.run(f"source ./devel/setup.sh && roslaunch arena_bringup start_arena.launch tm_robots:=scenario tm_obstacles:=scenario model:={model} local_planner:={planner} map_file:=map_custom record_data:=true", shell=True)
    
    f = FileCreator(base_path)
    items = os.listdir(f'{base_path}/src/arena/evaluation/arena-evaluation/data') 
    parentFolder = items[0]
    items = os.listdir(f"{base_path}/src/arena/evaluation/arena-evaluation/data/{parentFolder}")
    dataFolder = items[0]
    
    # TODO: Arena-Rosnav currently works on get_metrics. Finish when released
    #subprocess.run(f"source ./devel/setup.sh && python3 {base_path}/src/arena/evaluation/arena-evaluation/scripts/get_metrics.py {base_path}/src/arena/evaluation/arena-evaluation/data/{parentFolder}/{dataFolder}")

    zip64 = f.createBase64Zip(f"{base_path}/src/arena/evaluation/arena-evaluation/data/{parentFolder}")
    # avgCollisions = f.getAvgCollisions(f"{base_path}/src/arena/evaluation/arena-evaluation/data/{parentFolder}/{dataFolder}/metrics.py")
    # bestTime = f.getBestTime(f"{base_path}/src/arena/evaluation/arena-evaluation/data/{parentFolder}/{dataFolder}/metrics.py")

    response = {
        "taskId": taskId,
        "data": zip64,
        "best_time": random.randint(20, 160), # TODO: PLACEHOLDER RANDOM NUMBER! Finish when Arena-Rosnav functionalities are there
        "avg_collisions": random.randint(0, 5), # TODO: PLACEHOLDER RANDOM NUMBER! Finish when Arena-Rosnav functionalities are there
    }

else:

    # TODO: Arena-Rosnav currently works on create_plots. Finish when released
    #subprocess.run(f"source ./devel/setup.sh && python3 {base_path}/src/arena/evaluation/arena-evaluation/scripts/create_plots.py plot.yaml", shell=True)
    f = FileCreator(base_path)
    zip64 = f.createBase64Zip(f"{base_path}/src/arena/evaluation/arena-evaluation/plots")
    response = {
        "taskId": taskId,
        "data": zip64,
    }

print('Arena-Rosnav finished. Sending back data')

json_response = json.dumps(response)
sio.emit("taskFinished", json_response)