import sys
if sys.prefix == '/home/eunseop/anaconda3/envs/env_hand':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eunseop/nrs_media/install/nrs_mediapipe'
