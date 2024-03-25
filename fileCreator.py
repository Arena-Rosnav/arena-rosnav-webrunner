import os
import io
import csv
import json
import yaml
import zipfile
import base64
from PIL import Image
from pathlib import Path

class FileCreator:

    def __init__(self, base_path):
        self.base_path = base_path

    def createRobotFiles(self, configFiles):
        
        if not configFiles.get('robot'):
            return
        
        robotModelFile = configFiles.get('robot').get('modelFile')
        robotModelParams = configFiles.get('robot').get('modelParams')
        robotGlobalCostMap = configFiles.get('robot').get('globalCostMap')
        robotLocalCostMap = configFiles.get('robot').get('localCostMap')
        robotName = robotModelParams.get('robot_model')

        Path(f"{self.base_path}/src/arena/simulation-setup/entities/robots/{robotName}/yaml").mkdir(parents=True)
        Path(f"{self.base_path}/src/arena/simulation-setup/entities/robots/{robotName}/configs/costmaps").mkdir(parents=True)

        with open(f"{self.base_path}/src/arena/simulation-setup/entities/robots/model_params.yaml", "w") as yaml_file:
            yaml.dump(robotModelParams, yaml_file)

        with open(f"{self.base_path}/src/arena/simulation-setup/entities/robots/{robotName}.model.yaml", "w") as yaml_file:
            yaml.dump(robotModelFile, yaml_file)

        with open(f"{self.base_path}/src/arena/simulation-setup/entities/robots/yaml/{robotName}.model.yaml", "w") as yaml_file:
            yaml.dump(robotModelFile, yaml_file)

        with open(f"{self.base_path}/src/arena/simulation-setup/entities/robots/{robotName}/configs/costmaps/local_costmap_params.yaml", "w") as yaml_file:
            yaml.dump(robotLocalCostMap, yaml_file)

        with open(f"{self.base_path}/src/arena/simulation-setup/entities/robots/{robotName}/configs/costmaps/global_costmap_params.yaml", "w") as yaml_file:
            yaml.dump(robotGlobalCostMap, yaml_file)

    def createHyperparamsFile(self, configFiles):

        hyperparams = configFiles.get("hyperparamsFile")

        with open(f"{self.base_path}/src/arena/arena-rosnav/arena_bringup/configs/training/training_config.yaml", "w") as yaml_file:
            yaml.dump(hyperparams, yaml_file)

    def createWorldFile(self, configFiles):

        mapFile = configFiles.get('map').get('mapFile')
        mapWorldFile = configFiles.get('map').get('mapWorldFile')
        mapImg = configFiles.get('map').get('mapImg')

        Path(f"{self.base_path}/src/arena/simulation-setup/worlds/map_custom/map").mkdir(parents=True)

        with open(f"{self.base_path}/src/arena/simulation-setup/worlds/map_custom/map/map.yaml", "w") as yaml_file:
            yaml.dump(mapFile, yaml_file)

        with open(f"{self.base_path}/src/arena/simulation-setup/worlds/map_custom/map/map.world.yaml", "w") as yaml_file:
            yaml.dump(mapWorldFile, yaml_file)

        image_bytes = base64.b64decode(mapImg)
        image = Image.open(io.BytesIO(image_bytes))
        image.save(f"{self.base_path}/src/arena/simulation-setup/worlds/map_custom/map/map.png", format="PNG")

    def createScenarioFile(self, configFiles):

        scenarioFile = configFiles.get('scenarioFile')

        Path(f"{self.base_path}/src/arena/simulation-setup/worlds/map_custom/scenarios").mkdir(parents=True)

        with open(f"{self.base_path}/src/arena/simulation-setup/worlds/map_custom/scenarios/default.json", "w") as json_file:
            json.dump(scenarioFile, json_file)

    def createPlotFile(self, configFiles):

        plotFile = configFiles.get('plotFile')

        with open(f"{self.base_path}/src/arena/evaluation/arena_evaluation/plot_declarations/plot.yaml", "w") as yaml_file:
            yaml.dump(plotFile, yaml_file)

    def createBase64Zip(self, folderPath):
        zipBuffer = io.BytesIO()
        with zipfile.ZipFile(zipBuffer, 'a', zipfile.ZIP_DEFLATED, False) as zipFile:
            for root, dirs, files in os.walk(folderPath):
                for file in files:
                    filePath = os.path.join(root, file)
                    zipFile.write(filePath, os.path.relpath(filePath, folderPath))
        
        zipBuffer.seek(0)
        zipFile = zipBuffer.getvalue()

        return base64.b64encode(zipFile)
    
    def getAvgCollisions(self, metricsPath):
        collision_values = []
        with open(metricsPath, 'r', newline='') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                if "collision_amount" in row:
                    collision_values.append(float(row['collision_amount']))
        
        return ( sum(collision_values) / len(collision_values) )
    
    def getBestTime(self, metricsPath):
        best_time = float(160)
        with open(metricsPath, 'r', newline='') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                if "time_diff" in row:
                    if (float(row['time_diff']) < best_time):
                        best_time = float(row['time_diff'])
        return best_time