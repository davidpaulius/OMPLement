import os
import sys
import time
import math
from random import randint, shuffle

from utils import Interfacer

from scipy.interpolate import CubicSpline
import numpy as np

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    print(' -- ERROR: Set up CoppeliaSim ZeroMQ client as described here: '\
          'https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm')
    sys.exit()

robot_name = "Panda"

sim_interfacer = Interfacer(
    scene_file_name='./panda_stacking.ttt',
    robot_name=robot_name,
    robot_gripper=f"{robot_name}_gripper",
)

all_blocks = ["D_block", "C_block", "B_block", "A_block", ]

sim_interfacer.sim_start()

# -- do series of block-stacking:
for x in range(1, len(all_blocks)):
    success = sim_interfacer.pick(
        target_object=all_blocks[x],
        ompl_args={
            "ompl_state_resolution": float("3.0e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_orientation_constraint": None,
            "ompl_max_compute": 15,
            "ompl_max_simplify": 15,
        })

    success = sim_interfacer.place(
        target_object=all_blocks[x-1],
        ompl_args={
            "ompl_state_resolution": float("3.0e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            # NOTE: set "ompl_orientation_constraint" to None if you don't care about the orientation of gripper during place: 
            # "ompl_orientation_constraint": None,
            "ompl_orientation_constraint": "z",
            "ompl_orientation_threshold": 0.9,
            "ompl_max_compute": 15,
            "ompl_max_simplify": 15,
        },
        affordance='place-top')

sim_interfacer.return_home()

sim_interfacer.sim_pause()

pass