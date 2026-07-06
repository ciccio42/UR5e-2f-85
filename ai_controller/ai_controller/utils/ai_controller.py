from abc import ABC, abstractmethod

class AIController(ABC):
    """Abstract class for AI controller. 
    All AI controllers should inherit from this class and implement the following methods:
    - `__init__`: Initialize the controller.
    - `load_model`: Load the model from the given configuration.
    - `reset`: Reset the controller to its initial state.
    - `pre_process`: Pre-process the input data before feeding it to the model.
    - `post_process`: Post-process the output data from the model before sending it to the robot.
    - `inference`: Perform inference using the model.
    """
    
    def __init__(self, model_config: str):
        """Initialize the controller.
        
        Args:
            model_config (str): The configuration for the model.
        """
        self.model_config_path = model_config
        self.model = self.load_model(self.model_config_path)

    @abstractmethod
    def load_model(self, model_config: str):
        """Load the model from the given configuration.
        
        Args:
            model_config (str): The configuration for the model.
        """
        
        raise NotImplementedError("The method load_model() is not implemented.")
    
    @abstractmethod
    def move_model_to_device(self, device):
        """Move the model to the specified device.
        
        Args:
            device: The device to move the model to.
        """
        
        raise NotImplementedError("The method move_model_to_device() is not implemented.")
    
    @abstractmethod
    def reset(self):        
        """Reset the controller to its initial state."""
        raise NotImplementedError("The method reset() is not implemented.")
    
    @abstractmethod
    def pre_process(self, input_data):
        """Pre-process the input data before feeding it to the model.
        
        Args:
            input_data: The input data to be pre-processed.
        """
        raise NotImplementedError("The method pre_process() is not implemented.")
    
    @abstractmethod
    def post_process(self, output_data):
        """Post-process the output data from the model before sending it to the robot.
        
        Args:
            output_data: The output data from the model to be post-processed.
        """
        raise NotImplementedError("The method post_process() is not implemented.")
    
    @abstractmethod
    def inference(self, input_data):
        """Perform inference using the model.
        
        Args:
            input_data: The input data for inference.
        """
        raise NotImplementedError("The method inference() is not implemented.")