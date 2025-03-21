import os
import sys
from random import randint, shuffle
from typing import Type

from utils import Interfacer

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    print(' -- ERROR: Set up CoppeliaSim ZeroMQ client as described here: '\
          'https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm')
    sys.exit()

robot_name = "Panda"

sim_interfacer = Interfacer(
    # scene_file_name='./panda_cocktail.ttt',
    # scene_file_name="C:/Users/david/OLP_LLM/utamp/scenes/panda_blocks_randomized_2025-02-27_13-14-46S.ttt",
    scene_file_name="C:/Users/david/OLP_LLM/utamp/scenes/panda_blocks_randomized_2025-03-21_15-40-42S.ttt",
    robot_name=robot_name,
    robot_gripper=f"{robot_name}_gripper",
)

all_blocks = ["B_block_1", "D_block_1", "C_block_1", "P_block_1"]
shuffle(all_blocks)

setting = randint(0,1)
setting = 0

sim_interfacer.sim_start()

if setting == 2:
    # -- do series of block-stacking:
    for x in range(1, len(all_blocks)):
        success = sim_interfacer.pick(
            target_object=all_blocks[x],
            ompl_args={
                "ompl_state_resolution": float("2.5e-3"),
                "ompl_use_lua": True,
                "ompl_algorithm": "RRTConnect",
                "ompl_motion_constraint": "free",
                "ompl_max_compute": 15,
                "ompl_max_simplify": 15,
            })

        success = sim_interfacer.place(
            target_object=all_blocks[x-1],
            ompl_args={
                "ompl_state_resolution": float("2.5e-3"),
                "ompl_use_lua": True,
                "ompl_algorithm": "RRTConnect",
                "ompl_motion_constraint": "z",
                "ompl_max_compute": 15,
                "ompl_max_simplify": 15,
            },
            affordance='place-top')
elif setting == 2:
    success = sim_interfacer.pick(
        target_object="bottle_vodka",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "free",
            "ompl_max_compute": 5,
        },
        affordance='pick-side',
    )

    if not success:
        print("ERROR: no object in hand for pouring!")

    success = sim_interfacer.pour(
        source_container="bottle_vodka",
        target_container="Cup",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "z",
            "ompl_max_compute": 15,
        },
    )

    success = sim_interfacer.place(
        target_object="table",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "z",
            "ompl_max_compute": 15,
        },
    )

    # success = place(
    #     target_object="table",
    #     ompl_args={
    #         "ompl_state_resolution": float("2.5e-3"),
    #         "ompl_use_lua": True,
    #         "ompl_algorithm": "RRTConnect",
    #         "ompl_motion_constraint": "x",
    #         "ompl_max_compute": 15,
    #         "ompl_max_simplify": 15,
    #     },
    #     affordance='place-top',
    # )

    # success = pick(
    #     target_object="drinking_glass",
    #     ompl_args={
    #         "ompl_state_resolution": float("2.5e-3"),
    #         "ompl_use_lua": True,
    #         "ompl_algorithm": "RRTConnect",
    #         "ompl_motion_constraint": "free",
    #         "ompl_max_compute": 15,
    #         "ompl_max_simplify": 15,
    #     },
    #     affordance='pick-side',
    # )

    # if not success: sys.exit("ERROR: no object in hand for pouring!")


    # success = pour(
    #     source_container="drinking_glass",
    #     target_container="drinking_glass_2",
    #     ompl_args={
    #         "ompl_state_resolution": float("2.5e-3"),
    #         "ompl_use_lua": True,
    #         "ompl_algorithm": "RRTConnect",
    #         "ompl_motion_constraint": "x",
    #         "ompl_max_compute": 15,
    #         "ompl_max_simplify": 15,
    #     },
    # )

    # success = place(
    #     target_object="table",
    #     ompl_args={
    #         "ompl_state_resolution": float("2.5e-3"),
    #         "ompl_use_lua": True,
    #         "ompl_algorithm": "RRTConnect",
    #         "ompl_motion_constraint": "x",
    #         "ompl_max_compute": 15,
    #         "ompl_max_simplify": 15,
    #     },
    #     affordance='place-top',
    # )

# success = sim_interfacer.pick(
#     target_object="bottle_vodka",
#     ompl_args={
#         "ompl_state_resolution": float("2.5e-3"),
#         "ompl_use_lua": True,
#         "ompl_algorithm": "RRTConnect",
#         "ompl_motion_constraint": "free",
#         "ompl_max_compute": 15,
#     })

success = sim_interfacer.pick(
    target_object="P_block_2",
    ompl_args={
        "ompl_state_resolution": float("2.5e-3"),
        "ompl_use_lua": True,
        "ompl_algorithm": "RRTConnect",
        "ompl_motion_constraint": "free",
        "ompl_max_compute": 5,
    })

success = sim_interfacer.place(
    target_object="T_block_1",
    ompl_args={
        "ompl_state_resolution": float("2.5e-3"),
        "ompl_use_lua": True,
        "ompl_algorithm": "RRTConnect",
        "ompl_motion_constraint": "z",
        "ompl_max_compute": 15,
    })

# success = sim_interfacer.pick(
#     target_object="S_block_1",
#     ompl_args={
#         "ompl_state_resolution": float("2.5e-3"),
#         "ompl_use_lua": True,
#         "ompl_algorithm": "RRTConnect",
#         "ompl_motion_constraint": "free",
#         "ompl_max_compute": 5,
#     })

# success = sim_interfacer.place(
#     target_object="E_block_2",
#     ompl_args={
#         "ompl_state_resolution": float("2.5e-3"),
#         "ompl_use_lua": True,
#         "ompl_algorithm": "RRTConnect",
#         "ompl_motion_constraint": "z",
#         "ompl_max_compute": 15,
    # })

sim_interfacer.return_home()

sim_interfacer.sim_stop()
pass
