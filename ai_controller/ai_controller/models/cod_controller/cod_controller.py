import sys, os
import argparse
if __name__ == "__main__":
    sys.path.append(os.path.join(os.path.dirname(__file__), "../../.."))
from ai_controller.models.cod_controller.cond_target_obj_detector import CondTargetObjectDetector
from ai_controller.models.cod_controller.utils import build_tvf_formatter, move_to_device, seed_everything, denormalize_action, axisangle2quat
from ai_controller.utils.ai_controller import AIController
import hydra
from omegaconf import OmegaConf
import glob
import pickle
import importlib
import numpy as np
import PIL
import cv2
import torch
import copy

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
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        super().__init__(model_config)
        
        self.img_formatter = build_tvf_formatter(self.model_config_omega, env_name)
        # get normalization ranges for the action space from the model configuration
        self.action_ranges = self.model_config_omega.dataset_cfg.normalization_ranges
        print(f"Action normalization ranges: {self.action_ranges}")
        
        self.move_model_to_device(self.device) 
        # set random seed for reproducibility
        seed_everything(42)
        
        self.gripper_closed = False
        
    def load_model(self, model_config):
        """Load the COD model from the given configuration.
        
        Args:
            model_config: The configuration for the model.
        """
        self.model_config_omega = OmegaConf.load(model_config)
        model = hydra.utils.instantiate(self.model_config_omega.policy)
        
        # load the model weights from the specified checkpoint
        model_folder = os.path.dirname(model_config)
        models = glob.glob(os.path.join(model_folder, "model_save-*.pt"))
        # order the models by step number and select the latest one
        models = sorted(models, key=lambda x: int(x.split('-')[-1].split('.')[0]))
        if not models:
            raise FileNotFoundError(f"No model checkpoints found in {model_folder}.")
        model_path = models[-1]
        print(f"Loading model weights from {model_path}...")
         
        weights = torch.load(model_path, map_location=self.device)
        weights_copy = copy.deepcopy(weights)
        for key in weights_copy.keys():
            if 'object_detector' in key:
                del weights[key]
        model.load_state_dict(weights, strict=False)
        model.load_target_obj_detector(
                                        target_obj_detector_path=self.model_config_omega.policy.target_obj_detector_path,
                                        target_obj_detector_step=self.model_config_omega.policy.target_obj_detector_step
                                    )
        model._object_detector.eval()
        model.eval()    
        print("Model weights loaded successfully.")
        
        return model
        

    
    def move_model_to_device(self, device):
        """Move the model to the specified device.
        
        Args:
            device: The device to move the model to.
        """
        self.model.to(device)   
    
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
            # print(frame.shape)
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
        # 1. Save current trajectory data if needed
        # 2. Load the demo dataset for the current task
        self.gripper_closed = False
        
    
    def pre_process(self, input_data):
        """Pre-process the input data before feeding it to the model.
        
        Args:
            input_data: The input data to be pre-processed.
        """
        imgs = input_data[0]
        states = input_data[1]
        
        # apply the image formatter to the input image
        formatted_imgs = [self.img_formatter(img,
                                             agent=True) for img in imgs]
        
        if states is not None:
            raise NotImplementedError("State processing is not implemented yet.")
        else:
            formatted_states = None
        
        return [formatted_imgs, formatted_states]
    
    def post_process_bb(self, predicted_bb, target_obj_prediction):
        """Post-process the predicted bounding box from the model.
        
        Args:
            predicted_bb: The predicted bounding box from the model.
            target_obj_prediction: The target object prediction from the model.
        """
        pass
    
    def _post_process_action(self, action):
        """Post-process the predicted action from the model.
        
        Args:
            action: The predicted action from the model.
        """
        # print(f"Post-processing action: {action}")
        if len(action.shape) != 1:
            action_list = list()
            for t in range(action.shape[0]):
                # print(f"Post-processing action at time step {t}: {action[t]}")
                action_list.append(denormalize_action(action[t], self.action_ranges))
            action = action_list
        else:
            action = denormalize_action(action, self.action_ranges)
        
        desired_position = action[:3]
        desired_orientation = axisangle2quat(vec=action[3:6])
        predicted_gripper = action[-1]
        print(f"Predicted gripper value: {predicted_gripper}")
        if predicted_gripper > 0.50:
            gripper_finger_pos = 255
            self.gripper_closed = True
        else:
            gripper_finger_pos = 0
            
        action = np.concatenate([desired_position, desired_orientation, [gripper_finger_pos]])
        
        return action    
        
    
    def post_process(self, output_data):
        """Post-process the output data from the model before sending it to the robot.
        
        Args:
            output_data: The output data from the model to be post-processed.
        """ 
        action = output_data[0]
        predicted_bb = output_data[1]
        target_obj_prediction = output_data[2]
        
        # post-process the predicted action
        action = self._post_process_action(action)
        return action, predicted_bb, target_obj_prediction
    
    def inference(self, input_data, t, save_path=None):
        """Perform inference using the model.
        
        Args:
            input_data: The input data for inference.
            t: The time step for inference.
            save_path: The path to save the inference results.
        """

        print(f"Performing inference at time step {t}...")
        
        # pre-process the input data
        pre_processed_data = self.pre_process(input_data)
        pre_processed_imgs = pre_processed_data[0]
        pre_processed_states = pre_processed_data[1]
        
        # save pre-processed images to a folder for visualization
        if save_path is not None:
            os.makedirs(save_path, exist_ok=True)
            for i, img in enumerate(pre_processed_imgs):
                # convert the image to a numpy array and then to a PIL Image
                img_np = img.permute(1, 2, 0).cpu().numpy()  # Assuming img is a torch tensor
                pil_img = PIL.Image.fromarray((img_np * 255).astype(np.uint8))
                pil_img.save(os.path.join(save_path, f"pre_processed_img_{i}.png"))
        
        # move the pre-processed images to the device (e.g., GPU) if necessary
        # get only the frontal image for inference
        # print(f"Pre-processed images shape: {[img.shape for img in pre_processed_imgs]}")
        pre_processed_imgs, pre_processed_states, context = move_to_device(
                                                                pre_processed_imgs[0],
                                                                pre_processed_states,
                                                                self.demo_frames, 
                                                                device=self.device)
        
        # perform inference using the model
        with torch.no_grad():
            print(f"Performing model inference at time step {t} with gripper state: {self.gripper_closed}")
            self.model.first_phase = not(self.gripper_closed)
            out = self.model(
                        states=pre_processed_states,
                        images=pre_processed_imgs,
                        context=context,
                        bb=torch.zeros((1, 1, 2, 2), dtype=torch.float, device=self.device), # create a tensor of zeros with shape (2,2)
                        gt_classes=torch.zeros((1, 1, 2), dtype=torch.float, device=self.device), # create a tensor of zeros with the same shape as bb
                        predict_gt_bb=False,
                        eval=True,
                        target_obj_embedding=None,
                        compute_activation_map=True,
                        t=t)
            
            action = out['bc_distrib'].sample()[0, -1].cpu().numpy()
            predicted_bb = out.get('predicted_bb', None)
            target_obj_prediction = out.get('target_obj_prediction', None)
            # print(f"Inference output at time step {t}: 'action: {action}, predicted_bb: {predicted_bb}, target_obj_prediction: {target_obj_prediction}")
            
            # plot predictred_bb on the pre-processed image and save it to a folder for visualization
            if save_path is not None and predicted_bb is not None:
                # Implementation for plotting and saving the image
                img_np = pre_processed_imgs[0][0].permute(1, 2, 0).cpu().numpy()
                img_np = np.ascontiguousarray((img_np * 255).astype(np.uint8))
                # Convert predicted_bb to numpy array and scale to image dimensions
                for bb in predicted_bb[0][0].cpu().numpy():
                    x1, y1, x2, y2 = bb
                    cv2.rectangle(img_np, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                pil_img = PIL.Image.fromarray(img_np)
                pil_img.save(os.path.join(save_path, f"predicted_bb_img_{t}.png"))
                
                # show the image with predicted bounding boxes using OpenCV
                cv2.imshow(f"Predicted BB at step {t}", img_np[:, :, ::-1])  # Convert RGB to BGR for OpenCVclear
                cv2.waitKey(-1)  # Display the image for 1 s
                # close the OpenCV window
                cv2.destroyAllWindows()

            return self.post_process([action, predicted_bb, target_obj_prediction])
        
        
        
        
        

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
    