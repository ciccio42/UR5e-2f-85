from __future__ import annotations


prompt_tabletop_ui_ros = """
# Python robot control script

# Available scene APIs:
# - get_obj_pos(name): return the initial XY position of an object
# - get_obj_positions_np(names): return the initial XY positions of objects
# - get_obj_names(): return the names of all objects in the initial scene
# - is_obj_visible(name): check whether an object exists
# - parse_obj_name(description, context): resolve a natural-language object reference
# - parse_position(description, context): resolve a spatial position
# - say(message): print a message

# Available robot motion primitives:
# - reach(target): move above the target object
# - approaching(target): descend toward the target object
# - pick(target): grasp the target object
# - lift_up(target): lift the grasped object
# - moving(target): move toward the target destination
# - placing(target): place/release the object at the target destination

# IMPORTANT:
# Generate the robot program by calling the available motion primitives.
# Each primitive call is one independent robot-control step.
# Do not use put_first_on_second or any other composite pick-and-place API.
# Decide which primitives are necessary and in which order from the task.
# Use only objects that exist in the provided objects list.
#
# **OUTPUT FORMAT REQUIREMENTS — STRICT:**
# **Output ONLY valid executable Python code.**
# **Do NOT output explanations, descriptions, reasoning, introductions, or conclusions.**
# **Do NOT use Markdown code fences such as ```python or ```.**
# **Do NOT write natural-language text before or after the Python code.**
# **The first generated line must be valid Python code.**
# **The last generated line must be valid Python code.**
# **Use only calls to the available robot motion primitives.**
#
# Correct output example:
# reach('red cube')
# approaching('red cube')
# pick('red cube')
# lift_up('red cube')
# moving('first bin from the left')
# placing('first bin from the left')
#
# Incorrect output examples:
# "To pick up the red cube, follow these steps:"
# ```python
# reach('red cube')
# ```
# "This sequence will complete the task."

objects = [
    'red cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left'
]

# pick up the red cube and place it in the first bin from the left.
reach('red cube')
approaching('red cube')
pick('red cube')
lift_up('red cube')
moving('first bin from the left')
placing('first bin from the left')

objects = [
    'green cube',
    'yellow cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left'
]

# pick up the yellow cube and place it in the third bin from the left.
reach('yellow cube')
approaching('yellow cube')
pick('yellow cube')
lift_up('yellow cube')
moving('third bin from the left')
placing('third bin from the left')

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left'
]

# pick up the green cube and place it in the second bin from the left, and then pick up the blue cube and place it in the first bin from the left.
reach('green cube')
approaching('green cube')
pick('green cube')
lift_up('green cube')
moving('second bin from the left')
placing('second bin from the left')
reach('blue cube')
approaching('blue cube')
pick('blue cube')
lift_up('blue cube')
moving('first bin from the left')
placing('first bin from the left')

# FINAL REMINDER:
# **For the next task, respond with Python statements only.**
# **No prose. No Markdown. No code fences. No explanation.**
""".strip()

prompt_parse_obj_name_ros = """
import numpy as np
from env_utils import get_obj_pos, get_obj_names
from utils import get_obj_positions_np

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

# the red cube.
ret_val = 'red cube'

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

# the cubes.
ret_val = [
    'red cube',
    'green cube',
    'blue cube'
]

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

# the bins.
ret_val = [
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

# the first bin from the left.
bin_names = [
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]
bin_positions = get_obj_positions_np(bin_names)
ordered_indices = np.argsort(bin_positions[:, 0])
ret_val = bin_names[ordered_indices[0]]

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

# the second bin from the left.
bin_names = [
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]
bin_positions = get_obj_positions_np(bin_names)
ordered_indices = np.argsort(bin_positions[:, 0])
ret_val = bin_names[ordered_indices[1]]

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

# the third bin from the left.
bin_names = [
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]
bin_positions = get_obj_positions_np(bin_names)
ordered_indices = np.argsort(bin_positions[:, 0])
ret_val = bin_names[ordered_indices[2]]

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

# the fourth bin from the left.
bin_names = [
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]
bin_positions = get_obj_positions_np(bin_names)
ordered_indices = np.argsort(bin_positions[:, 0])
ret_val = bin_names[ordered_indices[3]]

objects = [
    'red cube',
    'green cube',
    'blue cube',
    'first bin from the left',
    'second bin from the left',
    'third bin from the left',
    'fourth bin from the left'
]

# the cube closest to the first bin from the left.
cube_names = [
    'red cube',
    'green cube',
    'blue cube'
]
cube_positions = get_obj_positions_np(cube_names)
target_position = get_obj_pos(
    'first bin from the left'
)
distances = np.linalg.norm(
    cube_positions - target_position,
    axis=1,
)
ret_val = cube_names[int(np.argmin(distances))]
""".strip()