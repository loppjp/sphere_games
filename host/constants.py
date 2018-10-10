from geometry_msgs.msg import Point

# Game runs for 5 minutes
TOTAL_ALLOWED_TIME = 300 # Seconds

# Attribute of camera image
PICTURE_SIZE = (1280, 960)

# Sites may need to adjust the following 4 settings to match their
# arena and camera positions
ORIGIN_PIXELS = Point(623, 493, 0)
RED_BASE = Point(224, 880, 0)
BLUE_BASE = Point(1017, 104, 0)

ARENA_WIDTH_PIXELS = 1021 - 205 # assumed square
ARENA_WIDTH_MM = 1143           # assumed square
WALL_TO_BASE = 70               # Assume square, and that bases are in corners, fudge factor included

ARENA_BOUNDS = {'left': min(RED_BASE.x, BLUE_BASE.x) - WALL_TO_BASE,
                'right': max(RED_BASE.x, BLUE_BASE.x) + WALL_TO_BASE,
                'top': min(RED_BASE.y, BLUE_BASE.y) - WALL_TO_BASE,
                'bottom': max(RED_BASE.y, BLUE_BASE.y) + WALL_TO_BASE}

FILTER_THRESHOLD = 60 # Distance in mm

# These should be good enough if arena built-to-spec
COVERT_PIXEL2MM = (ARENA_WIDTH_MM/10.) / (ARENA_WIDTH_PIXELS/10.)
COVERT_MM2PIXEL = (ARENA_WIDTH_PIXELS/10.) / (ARENA_WIDTH_MM/10.)
