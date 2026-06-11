import sys, os
import argparse
if __name__ == "__main__":
    sys.path.append(os.path.join(os.path.dirname(__file__), "../../.."))
from ai_controller.models.cod_controller.cond_target_obj_detector import CondTargetObjectDetector
from ai_controller.utils.ai_controller import AIController
import hydra
from omegaconf import OmegaConf

class CODController(AIController):
    """
    A controller for the COD (Conditioned-Object Detector) model.
    """
    def __init__(self, model_config):
        """Initialize the CODController.
        """
        super().__init__(model_config)
        
    def load_model(self, model_config):
        """Load the COD model from the given configuration.
        
        Args:
            model_config: The configuration for the model.
        """
        self.model_config_omega = OmegaConf.load(model_config)
        return hydra.utils.instantiate(self.model_config_omega.policy)

    def reset(self):
        """Reset the controller to its initial state."""
        pass
    
    def pre_process(self, input_data):
        """Pre-process the input data before feeding it to the model.
        
        Args:
            input_data: The input data to be pre-processed.
        """
        pass
    
    def post_process(self, output_data):
        """Post-process the output data from the model before sending it to the robot.
        
        Args:
            output_data: The output data from the model to be post-processed.
        """
        pass
    
    def inference(self, input_data):
        """Perform inference using the model.
        
        Args:
            input_data: The input data for inference.
        """
        pass

if __name__ == "__main__":
    args = argparse.ArgumentParser(description="Test the CODController.")
    args.add_argument("--model_config", type=str, default="config.yaml", help="The configuration for the model.")
    args.add_argument("--step", type=int, default=0, help="The step for the model.")
    
    args = args.parse_args()
    
    
    # # 1. Load the model configuration
    # print(f"Loading model configuration from {args.model_config}...")
    # config = OmegaConf.load(args.model_config)
    # print("Model configuration loaded:")
    
    # 2. Initialize the CODController with the loaded configuration
    print("Initializing CODController...")
    cod_controller = CODController(config)
    