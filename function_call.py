import os
import sys
import time
import math
from random import randint, shuffle

from driver import Interfacer

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
    scene_file_name='./panda_blocks_simple.ttt',
    robot_name=robot_name,
    robot_gripper=f"{robot_name}_gripper",
)

def pick(
    target_object: str,
    ompl_args: dict = {},
    affordance: str = 'pick-top',
):
    start_time = time.time()

    goal_achieved = False

    # -- we will try to find configs with OMPL and find a collision-avoiding plan up to a certain number of times
    for _ in range(3 if target_object in ['table', 'worksurface'] else 1):
        # -- we will be generous and allow three attempts at finding an empty place on the table:
        goal_poses = sim_interfacer.ompl_find_pose(target_object, affordance)

        for G in range(len(goal_poses)):
            # -- use the OMPL-based path planning method:
            success = sim_interfacer.ompl_path_planning(
                target_object,
                goal_poses[G],
                dict(ompl_args),
            )

            # -- check if we succeeded with this particular pose:
            if success:
                goal_achieved = goal_poses[G]
                break

        if bool(goal_achieved): break

    # -- if we could not find a plan, then we just return:
    if not goal_achieved: return False

    # -- close the gripper
    sim_interfacer.act_end_effector(action=1)

    time.sleep(0.5)

    # -- move the end-effector up:
    goal_achieved[2] += 0.1

    # NOTE: post-grasping check below:
    # -- get the object handles for the gripper's attach point:
    gripper_attachPoint = -1
    children = sim_interfacer.sim.getObjectsInTree(sim_interfacer.sim.getObject(f"/{sim_interfacer.robot_name}"))
    for C in children:
        if "attachPoint" in sim_interfacer.sim.getObjectAlias(C):
            gripper_attachPoint = C

    obj_in_hand = sim_interfacer.sim.getObjectChild(gripper_attachPoint, 0)
    if obj_in_hand > -1:
        # -- this means that we want to move the gripper up to remove the object from the top of the below object's surface:
        start = sim_interfacer.sim.getObjectPosition(sim_interfacer.sim.getObject(f'/{sim_interfacer.robot_name}/target'), sim_interfacer.sim.getObject(f'/{sim_interfacer.robot_name}')) + sim_interfacer.sim.getObjectOrientation(sim_interfacer.sim.getObject(f'/{sim_interfacer.robot_name}/target'), sim_interfacer.sim.getObject(f'/{sim_interfacer.robot_name}'))
        end = list(start)
        # -- we want to move up by half the height of the object:
        # end[2] += sim_interfacer.sim.getObjectFloatParam(obj_in_hand, sim_interfacer.sim.objfloatparam_objbbox_max_z)
        end[2] += 0.02

        # -- we also want to pick up the object and align it with the base of the robot:

        traj_move_up = sim_interfacer.spline_adjust_trajectory(
            sim_interfacer.generate_trajectory({'time': [0, 1], 'trajectory': [start, end]}, ntraj=25)
        )

        # -- execute the trajectory but do not change the state of the gripper:
        sim_interfacer.execute_trajectory(traj_move_up)

        if target_object in sim_interfacer.sim.getObjectAlias(obj_in_hand):
        # -- this means that we have successfully grasped the intended object:
            return True

    print(f'\t-- total time: {time.time() - start_time}')

    return False


def place(
    target_object: str,
    ompl_args: dict = {},
    affordance: str = 'place-top',
):
    start_time = time.time()

    goal_achieved = False

    # -- we will try to find configs with OMPL and find a collision-avoiding plan up to a certain number of times
    for _ in range(3 if target_object in ['table', 'worksurface'] else 1):
        # -- we will be generous and allow three attempts at finding an empty place on the table:
        goal_poses = sim_interfacer.ompl_find_pose(target_object, affordance)

        for goal in goal_poses:
            # -- use the OMPL-based path planning method:
            success = sim_interfacer.ompl_path_planning(
                target_object,
                goal,
                ompl_args,
            )

            # -- check if we succeeded with this particular pose:
            if success:
                goal_achieved = success
                break

        if goal_achieved: break

    # -- if we could not find a plan, then we just return:
    if not goal_achieved: return False

    # -- close the gripper
    sim_interfacer.act_end_effector(action=0)

    time.sleep(1)

    print(f'\t-- total time: {time.time() - start_time}')

    return True


def pour(
    source_container: str,
    target_container: str,
    ompl_args: dict = {},
):
    start_time = time.time()

    # NOTE: pouring comprises of the following subactions:
    #   1. pre-pouring: positioning a source container to a target container
    #   3. rotate the source container to transfer contents
    #   4. place the source container back somewhere

    # -- now, we need to find a path for pre-pouring:
    while True:
        result = sim_interfacer.ompl_path_planning(
            target_object=target_container,
            goal_pose=sim_interfacer.ompl_find_pose(
                target_object=target_container,
                affordance='pour',
            )[0],
            ompl_args=ompl_args,
        )

        if result: break

    target = sim_interfacer.sim.getObject(f'/{sim_interfacer.robot_name}/target')
    robot = sim_interfacer.sim.getObject(f'/{sim_interfacer.robot_name}')

    src_object_pos = sim_interfacer.sim.getObjectPosition(target, robot)
    tgt_object_pos = sim_interfacer.sim.getObjectPosition(sim_interfacer.sim.getObject(f"/{target_container}"), robot)

    # -- let's determine the direction of pouring:
    pour_direction = 'left' if src_object_pos[1] > tgt_object_pos[1] else 'right'

    # -- this is the maximum angle we will perform rotation:
    max_rotation = 120

    # -- get the initial pose of the gripper:
    start = sim_interfacer.sim.getObjectPose(target, robot)

    trajectory = [start]

    # -- rotate forward...
    for angle in range(max_rotation):
        end = sim_interfacer.sim.rotateAroundAxis(start, [1, 0, 0], start[:3], (angle * math.pi/180) * (-1.0 if pour_direction == "right" else 1.0))
        sim_interfacer.sim.setObjectPose(target, end, robot)
        trajectory.append(list(end))
        time.sleep(0.001)

    time.sleep(1)
    trajectory.reverse()

    # ... and back!
    for pose in trajectory:
        sim_interfacer.sim.setObjectPose(target, pose, robot)
        time.sleep(0.001)

    print(f'\t-- total time: {time.time() - start_time}')

    return True


all_blocks = ["B_block_1", "D_block_1", "C_block_1", "P_block_1"]
shuffle(all_blocks)

setting = randint(0,1)
setting = 0

sim_interfacer.start()

if bool(setting):
    for obj in all_blocks:
        success = sim_interfacer.execute(
            target_object=obj,
            ompl_args={
                "ompl_state_resolution": float("5.0e-3"),
                "ompl_use_lua": True,
                "ompl_algorithm": "RRTConnect",
                "ompl_motion_constraint": "y",
            },
            gripper_action=0
        )
else:
    print("stacking")

    # -- do series of block-stacking:
    for x in range(1, len(all_blocks)):
        success = pick(
            target_object=all_blocks[x],
            ompl_args={
                "ompl_state_resolution": float("2.5e-3"),
                "ompl_use_lua": True,
                "ompl_algorithm": "RRTConnect",
                "ompl_motion_constraint": "free",
                "ompl_max_compute": 15,
                "ompl_max_simplify": 15,
            })

        success = place(
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

    success = pick(
        target_object="bottle_vodka",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "free",
            "ompl_max_compute": 15,
            "ompl_max_simplify": 15,
        },
        affordance='pick-side',
    )

    if not success: sys.exit("ERROR: no object in hand for pouring!")

    success = pour(
        source_container="bottle_vodka",
        target_container="drinking_glass",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "x",
            "ompl_max_compute": 15,
            "ompl_max_simplify": 15,
        },
    )

    success = place(
        target_object="table",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "x",
            "ompl_max_compute": 15,
            "ompl_max_simplify": 15,
        },
        affordance='place-top',
    )

    success = pick(
        target_object="drinking_glass",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "free",
            "ompl_max_compute": 15,
            "ompl_max_simplify": 15,
        },
        affordance='pick-side',
    )

    if not success: sys.exit("ERROR: no object in hand for pouring!")


    success = pour(
        source_container="drinking_glass",
        target_container="drinking_glass_2",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "x",
            "ompl_max_compute": 15,
            "ompl_max_simplify": 15,
        },
    )

    success = place(
        target_object="table",
        ompl_args={
            "ompl_state_resolution": float("2.5e-3"),
            "ompl_use_lua": True,
            "ompl_algorithm": "RRTConnect",
            "ompl_motion_constraint": "x",
            "ompl_max_compute": 15,
            "ompl_max_simplify": 15,
        },
        affordance='place-top',
    )

sim_interfacer.stop()
pass
