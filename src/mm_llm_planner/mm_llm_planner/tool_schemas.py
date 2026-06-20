"""
Canonical tool specs for the Phase 4 tool_server services + task_complete.

One spec per tool: name, description, JSON-schema parameters, and how to
dispatch it as a ROS service call. Converters below turn this single list
into Anthropic's and OpenAI's native tool-calling formats so both providers
stay in sync with one source of truth.
"""

from mm_interfaces.srv import (
    CloseGripper,
    DetectObjects,
    DriveDistance,
    GetSceneState,
    MoveArmToNamed,
    MoveArmToPose,
    NavigateTo,
    OpenGripper,
    Pick,
    Place,
)

# Tools that change the world enough to warrant a fresh get_scene_state
# right after they run, so the LLM always reasons on current reality.
RESCAN_AFTER = {'navigate_to', 'drive_distance', 'pick', 'place'}

TOOLS = [
    {
        'name': 'navigate_to',
        'description': (
            'Drive the mobile base to a named location in the warehouse. '
            'Blocks until arrival or failure (e.g. Nav2 timeout).'
        ),
        'parameters': {
            'type': 'object',
            'properties': {
                'location': {
                    'type': 'string',
                    'description': (
                        'One of the known location names (see KNOWN LOCATIONS '
                        'in the scene state), e.g. "table", "dispatch_zone".'
                    ),
                },
            },
            'required': ['location'],
        },
        'ros_service': 'tool_server/navigate_to',
        'ros_type': NavigateTo,
        'request_fields': ['location'],
    },
    {
        'name': 'drive_distance',
        'description': (
            'Drive the base forward or backward by a relative distance in a '
            'straight line, in metres (positive = forward, negative = '
            'backward). Use this for small local adjustments — e.g. "move '
            'forward 1m" — that are NOT one of the known named locations. '
            'This is blind/dead-reckoning (no obstacle avoidance, unlike '
            'navigate_to) and capped at 2m per call for safety; for longer '
            'moves or moves near obstacles, use navigate_to instead.'
        ),
        'parameters': {
            'type': 'object',
            'properties': {
                'distance_m': {
                    'type': 'number',
                    'description': 'Metres to drive; positive=forward, negative=backward. Max magnitude 2.0.',
                },
            },
            'required': ['distance_m'],
        },
        'ros_service': 'tool_server/drive_distance',
        'ros_type': DriveDistance,
        'request_fields': ['distance_m'],
    },
    {
        'name': 'detect_objects',
        'description': (
            'Run the camera-based object detector and return all currently '
            'visible objects with their colour, label, 3D position, and '
            'confidence. Only finds objects within camera view of the '
            'current robot pose.'
        ),
        'parameters': {'type': 'object', 'properties': {}},
        'ros_service': 'tool_server/detect_objects',
        'ros_type': DetectObjects,
        'request_fields': [],
    },
    {
        'name': 'move_arm_to_named',
        'description': (
            'Move the arm to a pre-defined named pose. "home" tucks the arm '
            'in for safe navigation. "ready" is a moderate upright reaching '
            'pose used as a staging point before pick/place.'
        ),
        'parameters': {
            'type': 'object',
            'properties': {
                'pose_name': {'type': 'string', 'enum': ['home', 'ready']},
            },
            'required': ['pose_name'],
        },
        'ros_service': 'tool_server/move_arm_to_named',
        'ros_type': MoveArmToNamed,
        'request_fields': ['pose_name'],
    },
    {
        'name': 'move_arm_to_pose',
        'description': (
            'Move the gripper to a specific (x, y, z) position in the odom '
            'frame, in metres. Low-level positioning tool — prefer pick/place '
            'for grasping objects, since they include the correct approach '
            'and standoff distances.'
        ),
        'parameters': {
            'type': 'object',
            'properties': {
                'x': {'type': 'number'},
                'y': {'type': 'number'},
                'z': {'type': 'number'},
            },
            'required': ['x', 'y', 'z'],
        },
        'ros_service': 'tool_server/move_arm_to_pose',
        'ros_type': MoveArmToPose,
        'request_fields': ['x', 'y', 'z'],
    },
    {
        'name': 'pick',
        'description': (
            'Pick up a named object: moves above it, descends, closes the '
            'gripper, and lifts. The object must currently be visible/known '
            '(via detect_objects) and within arm reach of the current robot '
            'position — navigate to the right location first. Fails if the '
            'object is out of reach or the gripper ends too far from it.'
        ),
        'parameters': {
            'type': 'object',
            'properties': {
                'object_label': {
                    'type': 'string',
                    'description': 'Exact label from detect_objects, e.g. "red_cube".',
                },
            },
            'required': ['object_label'],
        },
        'ros_service': 'tool_server/pick',
        'ros_type': Pick,
        'request_fields': ['object_label'],
    },
    {
        'name': 'place',
        'description': (
            'Place the currently held object at a named location. Fails if '
            'no object is currently held (call pick first).'
        ),
        'parameters': {
            'type': 'object',
            'properties': {
                'location_name': {
                    'type': 'string',
                    'description': 'One of the known location names.',
                },
            },
            'required': ['location_name'],
        },
        'ros_service': 'tool_server/place',
        'ros_type': Place,
        'request_fields': ['location_name'],
    },
    {
        'name': 'open_gripper',
        'description': 'Open the gripper fully.',
        'parameters': {'type': 'object', 'properties': {}},
        'ros_service': 'tool_server/open_gripper',
        'ros_type': OpenGripper,
        'request_fields': [],
    },
    {
        'name': 'close_gripper',
        'description': 'Close the gripper fully (without the pick approach sequence).',
        'parameters': {'type': 'object', 'properties': {}},
        'ros_service': 'tool_server/close_gripper',
        'ros_type': CloseGripper,
        'request_fields': [],
    },
    {
        'name': 'get_state',
        'description': (
            'Get the current scene state on demand: robot pose, arm joints, '
            'detected objects, known locations. Normally you do not need to '
            'call this yourself — a fresh scene state is appended '
            'automatically after every navigate_to/pick/place.'
        ),
        'parameters': {'type': 'object', 'properties': {}},
        'ros_service': '/get_scene_state',
        'ros_type': GetSceneState,
        'request_fields': [],
    },
]

# task_complete is handled directly by planner_node (no ROS service call).
TASK_COMPLETE_TOOL = {
    'name': 'task_complete',
    'description': (
        'Call this when the task is fully done (or when it cannot be '
        'completed and you are giving up). Ends the loop.'
    ),
    'parameters': {
        'type': 'object',
        'properties': {
            'summary': {
                'type': 'string',
                'description': 'One or two sentences describing what was done (or why it failed).',
            },
            'success': {
                'type': 'boolean',
                'description': 'Whether the task was actually accomplished.',
            },
        },
        'required': ['summary', 'success'],
    },
}

ALL_TOOLS = TOOLS + [TASK_COMPLETE_TOOL]


def get_tool_spec(name: str):
    for t in ALL_TOOLS:
        if t['name'] == name:
            return t
    return None


def to_anthropic_tools():
    return [
        {
            'name': t['name'],
            'description': t['description'],
            'input_schema': t['parameters'],
        }
        for t in ALL_TOOLS
    ]


def to_openai_tools():
    return [
        {
            'type': 'function',
            'function': {
                'name': t['name'],
                'description': t['description'],
                'parameters': t['parameters'],
            },
        }
        for t in ALL_TOOLS
    ]


def to_gemini_function_declarations():
    return [
        {
            'name': t['name'],
            'description': t['description'],
            'parameters': t['parameters'],
        }
        for t in ALL_TOOLS
    ]
