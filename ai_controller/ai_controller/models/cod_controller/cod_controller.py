import sys, os
import argparse
if __name__ == "__main__":
    sys.path.append(os.path.join(os.path.dirname(__file__), "../../.."))
from ai_controller.models.cod_controller.cond_target_obj_detector import CondTargetObjectDetector
from ai_controller.models.cod_controller.utils import build_tvf_formatter
from ai_controller.utils.ai_controller import AIController
import hydra
from omegaconf import OmegaConf
import glob
import pickle
import importlib
import numpy as np
import PIL
import cv2

class TrajectoryUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith('multi_task_il') and name == 'Trajectory':
            return importlib.import_module('scripts.savers').Trajectory
        return super().find_class(module, name)

class CODController(AIController):
    """
    A controller for the COD (Conditioned-Object Detector) model.
    """
    def __init__(self, model_config, env_name):
        """Initialize the CODController.
        """
        super().__init__(model_config)
        
        
        self.img_formatter = build_tvf_formatter(self.model_config_omega, env_name)
        
    def load_model(self, model_config):
        """Load the COD model from the given configuration.
        
        Args:
            model_config: The configuration for the model.
        """
        self.model_config_omega = OmegaConf.load(model_config)
        return hydra.utils.instantiate(self.model_config_omega.policy)
    
    def select_random_frames(self, frames, n_select=4, sample_sides=True, random_frames=True):
        """Select a specified number of random frames from the given frames.
        Args:
            frames: The frames from which to select.
            n_select: The number of frames to select.
            sample_sides: Whether to sample frames from the sides.
            random_frames: Whether to sample frames randomly.
        """
        selected_frames_indx = []
        def clip(x): return int(max(0, min(x, len(frames) - 1)))
        per_bracket = max(len(frames) / n_select, 1)

        if random_frames:
            for i in range(n_select):
                n = clip(np.random.randint(
                    int(i * per_bracket), int((i + 1) * per_bracket)))
                if sample_sides and i == n_select - 1:
                    n = len(frames) - 1
                elif sample_sides and i == 0:
                    n = 1
                selected_frames_indx.append(n)
        else:
            for i in range(n_select):
                # get first frame
                if i == 0:
                    n = 1
                # get the last frame
                elif i == n_select - 1:
                    n = len(frames) - 1
                elif i == 1:
                    obj_in_hand = 0
                    # get the first frame with obj_in_hand and the gripper is closed
                    for t in range(1, len(frames)):
                        state = frames.get(t)['info']['status']
                        trj_t = frames.get(t)
                        gripper_act = trj_t['action'][-1]
                        if state == 'obj_in_hand' and gripper_act == 1:
                            obj_in_hand = t
                            n = t
                            break
                elif i == 2:
                    # get the middle moving frame
                    start_moving = 0
                    end_moving = 0
                    for t in range(obj_in_hand, len(frames)):
                        state = frames.get(t)['info']['status']
                        if state == 'moving' and start_moving == 0:
                            start_moving = t
                        elif state != 'moving' and start_moving != 0 and end_moving == 0:
                            end_moving = t
                            breakdata
                    n = start_moving + int((end_moving-start_moving)/2)
                selected_frames_indx.append(n)

        selected_frames = []
        for i in range(len(selected_frames_indx)):
            selected_frame = frames[selected_frames_indx[i]]['obs']['camera_front_image']
            if len(selected_frame.shape) == 4:
                selected_frames.append(selected_frame)
            elif len(selected_frame.shape) == 1:
                # decoce the image from bytes to numpy array
                selected_frame = cv2.imdecode(selected_frame, cv2.IMREAD_COLOR)
                selected_frames.append(selected_frame)
            else:
                raise ValueError(f"Unexpected shape for selected frame: {selected_frame.shape}")

        return selected_frames
        
    def load_demo_dataset(self, demo_path, task_id):
        """Load the demo dataset for the given task ID.
        
        Args:
            demo_path: The path to the demo dataset.
            task_id: The ID of the task for which to load the dataset.
        """
        demo_files = glob.glob(os.path.join(demo_path, f"task_{task_id}/*.pkl"))
        if not demo_files:
            raise FileNotFoundError(f"No demo files found for task ID {task_id} in {demo_path}.")
        
        # Load the first demo file found
        demo_file = demo_files[0]
        print(f"Loading demo dataset from {demo_file}...")
        # Here you would load the dataset (e.g., using pickle or another method)
        # For now, we'll just log that we would load it.
        print(f"Demo dataset loaded from {demo_file}.")
        
        # open pickle file 
        with open(demo_file, 'rb') as f:
            demo_data = TrajectoryUnpickler(f).load()
            demo_frames = demo_data['traj']
            
        print(f"Demo data loaded for task ID {task_id}. Selecting random frames for inference...")
        self.demo_frames = self.select_random_frames(demo_frames, 
                                                     n_select=4, 
                                                     sample_sides=True, 
                                                     random_frames=True)
        
        # save demo_frames with PIL Images to a folder for visualization
        demo_frames_folder = os.path.join(demo_path, f"task_{task_id}/demo_frames")
        os.makedirs(demo_frames_folder, exist_ok=True)
        for i, frame in enumerate(self.demo_frames):
            print(frame.shape)
            pil_img = PIL.Image.fromarray(frame)
            pil_img.save(os.path.join(demo_frames_folder, f"frame_{i}.png"))
        print(f"Selected demo frames saved to {demo_frames_folder}.")
        
        # apply the image formatter to the selected demo frames
        self.demo_frames = [self.img_formatter(frame) for frame in self.demo_frames]
        # save the formatted demo frames to a folder for visualization
        formatted_demo_frames_folder = os.path.join(demo_path, f"task_{task_id}/formatted_demo_frames")
        os.makedirs(formatted_demo_frames_folder, exist_ok=True)
        for i, frame in enumerate(self.demo_frames):
            # convert the frame to a numpy array and then to a PIL Image
            frame_np = frame.permute(1, 2, 0).cpu().numpy()  # Assuming frame is a torch tensor
            pil_img = PIL.Image.fromarray((frame_np * 255).astype(np.uint8))
            pil_img.save(os.path.join(formatted_demo_frames_folder, f"formatted_frame_{i}.png"))
        print(f"Formatted demo frames saved to {formatted_demo_frames_folder}.")
            
    

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
    