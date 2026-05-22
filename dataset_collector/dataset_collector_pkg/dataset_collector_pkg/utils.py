import cv2
EEF_POS_NAME='eef_pos'
EEF_QUAT_NAME='eef_quat'
JOINT_POS_NAME='joint_pos'
JOINT_VEL_NAME='joint_vel'
IMAGE_RGB_NAME='image_rgb'
IMAGE_DEPTH_NAME='image_depth'
GRIPPER_QPOS_NAME='gripper_qpos'
GRIPPER_QVEL_NAME='gripper_qvel'
EE_AA_NAME='eef_aa'
obj_loc = [-1, -1]

MAP_TASK_ID_OBJECT = {
    'pick_place': {
        0: ('greenbox', 'bin1'),
        1: ('greenbox', 'bin2'),
        2: ('greenbox', 'bin3'),
        3: ('greenbox', 'bin4'),
        4: ('yellowbox', 'bin1'),
        5: ('yellowbox', 'bin2'),
        6: ('yellowbox', 'bin3'),
        7: ('yellowbox', 'bin4'),
        8: ('bluebox', 'bin1'),
        9: ('bluebox', 'bin2'),
        10: ('bluebox', 'bin3'),
        11: ('bluebox', 'bin4'),
        12: ('redbox', 'bin1'),
        13: ('redbox', 'bin2'),
        14: ('redbox', 'bin3'),
        15: ('redbox', 'bin4'),
    }
}

OBJ_LIST = {
    'pick_place': ['greenbox', 'yellowbox', 'bluebox', 'redbox', 'bin1', 'bin2', 'bin3', 'bin4']
}

def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # print(f'Clicked at: ({x}, {y})')
        # Here you can save the coordinates to a dictionary or a file as needed
        # For example, you could append them to a list or save them in a global variable
        global obj_loc
        obj_loc = [x, y]

def get_object_centers(node, task_name, variation_id, camera_name, rgb_image, depth_image):

    global obj_loc
    obj_loc = []

    obj_bb = dict()
    obj_bb[camera_name] = dict()

    node.get_logger().info(f'Getting object centers for task {task_name} and variation {variation_id}...')

    # get the object name and bin name from the task id and variation id
    target_object_name, target_bin_name = MAP_TASK_ID_OBJECT[task_name][variation_id]
    node.get_logger().info(f'\tObject name: {target_object_name}, Bin name: {target_bin_name}')

    for obj_idx, obj_name in enumerate(OBJ_LIST[task_name]):
        if obj_name == target_object_name or obj_name == target_bin_name:
            node.get_logger().info(f'\tGetting center for target object {obj_name}...')
            # show image
        else:
            node.get_logger().info(f'\tGetting center for other object {obj_name}...')
        
        # show image and register mouse click callback to get the center of the object in the image and save it in a dictionary
        cv2.imshow('RGB Image', rgb_image)
        cv2.setMouseCallback('RGB Image', on_mouse_click)
        k = cv2.waitKey(0)
        cv2.destroyAllWindows()
        if k != 32:
            obj_loc = [-1, -1]

        # save the object center in the dictionary
        if obj_name not in obj_bb[camera_name]:
            obj_bb[camera_name][obj_name] = dict()

        obj_bb[camera_name][obj_name]['center'] = obj_loc
        obj_bb[camera_name][obj_name]['upper_left_corner'] = [-1, -1]
        obj_bb[camera_name][obj_name]['bottom_right_corner'] = [-1, -1]
        
    
    return obj_bb