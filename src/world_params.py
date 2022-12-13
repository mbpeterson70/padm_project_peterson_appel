import numpy as np

# Positions
# BASE_COUNTER_POSE = (0.71, 0.49+.2, np.pi/2)
BASE_COUNTER_POSE = (0.69, -1.1, np.pi)
POSE_ON_COUNTER = (.2, 1.09, 0.1) 
POSE_ON_BURNER = (-0.06-.11, 0.5+.11, .1) # (-0.06-.05, 0.5+.05, .1)
POSE_AWAY = (.03, .66, -.36)
POSE_AWAY2 = (.35, 1.0, -.35)

# Orientations/Attitudes
ATT_AT_HANDLE = (.5, .5, -.5, -.5)
ATT_ON_COUNTER = (.924, .383, 0, 0) #(-.707, 0, .707, 0)
ATT_ON_BURNER = (.653, -.271, -.653, -.271) #(.924, -.383, 0, 0)
ATT_AWAY = ATT_ON_BURNER
ATT_AWAY2 = (.707, 0, 0, -.707)

# Link Names
HANDLE_NAME = 'indigo_drawer_handle_top'
DRAWER_NAME = 'indigo_drawer_top'
SPAM_NAME = 'potted_meat_can1'
SUGAR_NAME = 'sugar_box0'

# Opening Drawer
DRAWER_OPEN_DIST = .4
DRAWER_OPEN_SCALE = .9
DRAWER_INTERP_NUM = 20
DRAWER_OPEN_DIR = (1, 0, 0)