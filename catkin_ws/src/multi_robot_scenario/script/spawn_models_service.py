
#!/usr/bin/env python
import rospy
from multi_robot_scenario.srv import SpawnModels, SpawnModelsResponse
import os

class ModelSpawner:
    def __init__(self):
        self.service = rospy.Service('spawn_models', SpawnModels, self.handle_spawn_models)

    def spawn_models(self, model_name, x, y, z, num_models, sdf_file_path):
        for i in range(int(num_models)):
            name = f"{model_name}_{i}"  # Use unique names for each model
            sdf_file_path = os.path.expanduser(sdf_file_path)
            # Call the new function to spawn the model using SDF
            self.spawn_model_sdf(name, x, y, z, sdf_file_path)
        return True, "Models spawned successfully."

    def handle_spawn_models(self, req):
        success, message = self.spawn_models(req.model_name, req.x, req.y, req.z, req.num_models, req.sdf_file_path)
        return SpawnModelsResponse(success, message)

if __name__ == "__main__":
    rospy.init_node('model_spawner_service')
    ms = ModelSpawner()
    rospy.spin()