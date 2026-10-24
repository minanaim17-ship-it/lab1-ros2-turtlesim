"""Single place for every turtlesim name / number used by the nodes."""

TURTLE = 'turtle1'
POSE_TOPIC = f'/{TURTLE}/pose'
CMD_VEL_TOPIC = f'/{TURTLE}/cmd_vel'
CLEAR_SERVICE = '/clear'
TELEPORT_SERVICE = f'/{TURTLE}/teleport_absolute'
SET_PEN_SERVICE = f'/{TURTLE}/set_pen'

# turtlesim world and the pose the turtle starts with
WORLD_MIN = 0.0
WORLD_MAX = 11.0
SAFE_MARGIN = 0.3            # keep drawn shapes this far from the walls
START_X = 5.544445
START_Y = 5.544445
START_THETA = 0.0

# turtlesim default pen (restored after the silent teleport)
PEN_R, PEN_G, PEN_B, PEN_WIDTH = 179, 184, 255, 3
TELEPORT_SETTLE_SEC = 0.15   # let turtlesim's 60 Hz update loop consume the teleport

# speed limits
DEFAULT_MAX_LINEAR = 2.0     # units/s
DEFAULT_MAX_ANGULAR = 2.0    # rad/s
SPEED_UPPER_BOUND = 10.0

# goToPose server
CONTROL_RATE_HZ = 50.0
FEEDBACK_PERIOD_SEC = 0.1
GOAL_TIMEOUT_SEC = 60.0
