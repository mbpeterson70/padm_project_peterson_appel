import numpy as np

# Positions
BASE_COUNTER_POSE = (0.71, 0.49+.2, np.pi/2)

# Orientations/Attitudes
ATT_AT_HANDLE = (.5, .5, -.5, -.5)

# Link Names
HANDLE_NAME = 'indigo_drawer_handle_top'
DRAWER_NAME = 'indigo_drawer_top'

# Opening Drawer
DRAWER_OPEN_DIST = .5
DRAWER_OPEN_SCALE = .9
DRAWER_INTERP_NUM = 20
DRAWER_OPEN_DIR = (1, 0, 0)