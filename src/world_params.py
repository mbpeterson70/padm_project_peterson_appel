import numpy as np

# Positions
BASE_COUNTER_POSE = (0.71, 0.49+.2, np.pi/2)
POSE_ON_COUNTER = (.2, 1.1, 0.1) 
POSE_ON_BURNER = (-0.06, 0.5, -0.5554999999999999)

# Orientations/Attitudes
ATT_AT_HANDLE = (.5, .5, -.5, -.5)
ATT_ON_COUNTER = (-.707, 0, .707, 0)
ATT_ON_BURNER = ATT_ON_COUNTER

# Link Names
HANDLE_NAME = 'indigo_drawer_handle_top'
DRAWER_NAME = 'indigo_drawer_top'

# Opening Drawer
DRAWER_OPEN_DIST = .4
DRAWER_OPEN_SCALE = .9
DRAWER_INTERP_NUM = 20
DRAWER_OPEN_DIR = (1, 0, 0)