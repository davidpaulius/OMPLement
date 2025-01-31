import os
import sys
import time
import math

from scipy.interpolate import CubicSpline
import numpy as np

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    print(' -- ERROR: Set up CoppeliaSim ZeroMQ client as described here: '\
          'https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm')
    sys.exit()

client = RemoteAPIClient(host='localhost')

sim = client.require('sim')
# status = sim.loadScene(os.path.abspath('./panda_uibk_ffrob_problem32_close_blocks_ompl_david_bounds.ttt'))
status = sim.loadScene(os.path.abspath('./panda_blocks_simple.ttt'))

# -- loading required modules for simulation:
simIK = client.require('simIK')
simOMPL = client.require('simOMPL')

robot_name = "Panda"

def move_to_configs(path: list[float]):
    # NOTE: check the sim.moveToConfig() docs here: https://manual.coppeliarobotics.com/en/regularApi/simMoveToConfig.htm
    vel = 120
    accel = 60
    jerk = 60

    maxVel = [vel*math.pi/180, vel*math.pi/180, vel*math.pi/180, vel*math.pi/180, vel*math.pi/180, vel*math.pi/180, vel*math.pi/180]
    maxAccel = [accel*math.pi/180, accel*math.pi/180, accel*math.pi/180, accel*math.pi/180, accel*math.pi/180, accel*math.pi/180, accel*math.pi/180]
    maxJerk = [jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180]

    # -- get all the joint angles of the robot:
    joint_handles = []
    num_joints = 1

    while True:
        # -- using "noError" so default handle is -1 (if not found);
        #    read more here: https://manual.coppeliarobotics.com/en/regularApi/simGetObject.htm
        obj_handle = sim.getObject(f"/{robot_name}/joint", {"noError": True, "index":(num_joints-1)})

        if obj_handle == -1: break

        joint_handles.append(obj_handle)
        num_joints += 1

    # -- change the simulation setting to stepping mode for threaded/non-blocking execution:
    sim.setStepping(True)
    sim.step()

    # -- iterate through the entire plan of robot configurations:
    for P in range(len(path)):
        params = {
            'joints': joint_handles,
            'targetPos': path[P],
            # 'maxVel': maxVel,
            'targetVel': [0.8 * x for x in maxVel],
            # 'maxAccel': maxAccel,
            # 'maxJerk': maxJerk,
        }
        sim.moveToConfig(params)
        sim.step()

    # -- turn off stepping mode since we don't need it beyond this point:
    sim.setStepping(False)


def ompl_path_planning(
        target_object: str,
        goal_pose: list[float],
        ompl_args: dict = {},
        draw_path: bool = True,
        motion_method: str = "moveToConfig",
    ) -> bool:

    # -- formatting the string name for printing a cool message:
    if target_object:
        sim.addLog(sim.verbosity_default, f'[FOON-TAMP]: finding a plan to object "{target_object}"...')
        print(f'[FOON-TAMP]: finding a plan to object "{target_object}"...')

    # -- create a dummy object that will represent the target goal:
    target_goal = sim.createDummy(0.025)
    sim.setObjectPose(target_goal, goal_pose, sim.getObject(f'/{robot_name}'))
    sim.setObjectColor(target_goal, 0, sim.colorcomponent_ambient_diffuse, [0.0, 1.0, 0.0])
    sim.setObjectColor(target_goal, 0, sim.colorcomponent_emission, [0.6, 0.6, 0.6])
    sim.setObjectAlias(target_goal, 'OMPL_target')

    # -- we sleep for a bit so we can see this object appear in the sim:
    time.sleep(0.0001)

    # NOTE: checking if key parameters have been specified for OMPL:
    if not bool(ompl_args): ompl_args = {}
    if "ompl_algorithm" not in ompl_args:
        ompl_args["ompl_algorithm"] = "RRTConnect"
    try:
        ompl_args['ompl_algorithm'] = eval(f"simOMPL.Algorithm.{ompl_args['ompl_algorithm']}")
    except AssertionError:
        print(f"WARNING: {ompl_args['ompl_algorithm']} is not a valid algorithm!")
        ompl_args['ompl_algorithm'] = eval(f"simOMPL.Algorithm.RRTConnect")
    if "ompl_num_attempts" not in ompl_args:
        ompl_args["ompl_num_attempts"] = 5
    if "ompl_max_compute" not in ompl_args:
        ompl_args["ompl_max_compute"] = 15
    if "ompl_max_simplify" not in ompl_args:
        # NOTE: let OMPL do default simplification, signified by -1:
        ompl_args["ompl_max_simplify"] = -1
    if "ompl_len_path" not in ompl_args:
        # NOTE: let OMPL do give default number of configs in solution path, signified by 0:
        ompl_args["ompl_len_path"] = 0
    if "ompl_state_resolution" not in ompl_args:
        ompl_args["ompl_state_resolution"] = float("5.0e-3")
    if "ompl_use_state_validation" not in ompl_args:
        ompl_args["ompl_use_state_validation"] = True
    if "ompl_use_lua" not in ompl_args:
        ompl_args["ompl_use_lua"] = True
    if "ompl_motion_constraints" not in ompl_args:
        ompl_args["ompl_motion_constraints"] = "free"

    print(ompl_args)

    ompl_script = sim.getScript(sim.scripttype_simulation, sim.getObject('/OMPLement'))

    path, _ = sim.callScriptFunction(
        "ompl_path_planning",
        ompl_script,
        {
            "robot": robot_name,
            "goal": target_goal,
            "ompl_algorithm": ompl_args["ompl_algorithm"],
            "ompl_max_compute": ompl_args["ompl_max_compute"],
            "ompl_max_simplify": ompl_args["ompl_max_simplify"],
            "ompl_len_path": ompl_args["ompl_len_path"],
            "ompl_state_resolution": ompl_args["ompl_state_resolution"],
            "ompl_motion_constraints": ompl_args["ompl_motion_constraints"],
            "ompl_use_state_validation": ompl_args["ompl_use_state_validation"],
            "ompl_use_lua": ompl_args["ompl_use_lua"],
        },
    )

    if path:
        print(f'[FOON-TAMP]: plan found!')
        sim.addLog(sim.verbosity_default, f'[FOON-TAMP]: plan found!')

        # -- we need to disable the IK following done by the "target" dummy of the robot:
        # sim.setModelProperty(target, sim.modelproperty_scripts_inactive)
        ik_script = sim.getScript(sim.scripttype_simulation, sim.getObject(f"/{robot_name}"))
        if ik_script == -1:
            ik_script = sim.getScript(sim.scripttype_customization, sim.getObject(f"/{robot_name}"))

        sim.setObjectInt32Param(ik_script, sim.scriptintparam_enabled, 0)

        # -- if set to true, we will draw the path in the simulation:
        if draw_path: drawn_object = sim.callScriptFunction('visualizePath', ompl_script, path, [0.0, 1.0, 0.0])

        time.sleep(0.01)

        if motion_method != "moveToConfig":
            # -- use a cubic spline to interpolate time points:
            cs = CubicSpline(
                [0, 0.3, 0.5, 0.8, 1],
                [float('2.5e-3'), float('2.0e-3'), float('1.0e-3'), float('2.0e-3'), float('2.5e-3')]
            )
            xs = np.arange(0, 1, 1/len(path))
            time_points = cs(xs)

            # -- with the computed path, we will gradually change the configuration of the robot:
            for P in range(len(path)):
                sim.callScriptFunction('setConfig_python', ompl_script, path[P])
                time.sleep(time_points[P])
        else:
            move_to_configs(path)

        time.sleep(0.01)

        # -- we need to re-enable the IK following done by the "target" dummy of the robot:
        sim.setObjectPosition(sim.getObject(f'/{robot_name}/target'), sim.getObjectPosition(sim.getObject(f'/{robot_name}/tip')), -1)
        sim.setObjectOrientation(sim.getObject(f'/{robot_name}/target'), sim.getObjectOrientation(sim.getObject(f'/{robot_name}/tip')), -1)
        sim.setObjectInt32Param(ik_script, sim.scriptintparam_enabled, 1)
        if draw_path: sim.removeDrawingObject(drawn_object)

    else:
        sim.addLog(sim.verbosity_default, f'[FOON-TAMP]: plan not found!')
        print(f'[FOON-TAMP]: plan not found!')

    # -- remove the OMPL target object:
    sim.removeObjects([sim.getObject('/OMPL_target')])

    return bool(path)


def find_pose_for_ompl(
        robot_name: str,
        target_object: str,
    ) -> list[float]:

    robot = sim.getObject(f"/{robot_name}")
    target = sim.getObject(f"/{robot_name}/target")

    index = 0
    # # -- check if the target object refers to the table, as we will have to find an empty spot:
    # if target_object in ["table", "worksurface"]:
    #     empty_spot = choice(find_empty_spots())
    #     index, target_object = empty_spot['index'], sim.getObjectAlias(empty_spot['handle'])
    #     sim.addLog(sim.verbosity_default, f"table grounding: found empty spot: /{target_object}[{empty_spot['index']}]")
    #     if verbose:
    #         print(f"table grounding: found empty spot: /{target_object}[{empty_spot['index']}]")

    goal = sim.getObject(f"/{target_object}", {"index": index})

    candidate_goal_poses = []

    for rotate in [0.0, (math.pi)]:
        # -- Find a collision-free config that matches a specific pose:
        goal_pose = sim.getObjectPose(goal, robot)

        # -- get the object handles for the gripper's attach point:
        gripper_attachPoint = -1
        children = sim.getObjectsInTree(sim.getObject(f"/{robot_name}"))
        for C in children:
            if "attachPoint" in sim.getObjectAlias(C):
                gripper_attachPoint = C

        if gripper_attachPoint == -1:
            sys.exit("ERROR: robot gripper attach point was not found!")

        # -- determine the height based on whether there is an object in hand or not:
        obj_in_hand = sim.getObjectChild(gripper_attachPoint, 0)
        if obj_in_hand != -1:
            # -- first, we find a spot that sits RIGHT ON TOP of the surface...
            goal_pose[2] += sim.getObjectFloatParam(goal, sim.objfloatparam_objbbox_max_z)
            # ... then we will find a spot that considers the height of the object:
            goal_pose[2] += sim.getObjectFloatParam(obj_in_hand, sim.objfloatparam_objbbox_max_z) * (1.5 if target_object not in ["table", "worksurface"] else 1.25)

            # -- we also want to consider the orientation of the object
            orientation = sim.getObjectOrientation(goal, target)
            goal_pose[3:] = sim.buildPose(goal_pose[:3], [orientation[0], orientation[1], math.pi + rotate])[3:]

        else:
            # -- account for fingertip placement on object:
            goal_pose[2] += sim.getObjectFloatParam(goal, sim.objfloatparam_objbbox_max_z) * 3.0
            # -- try to match the orientation of the surface object:
            orientation = sim.getObjectOrientation(goal, robot)
            goal_pose[3:] = sim.buildPose(goal_pose[:3], [-(math.pi), orientation[1], orientation[2] + rotate])[3:]

        # NOTE: these are the ideal poses that Alejandro would use for picking from the side:
        # if obj_in_hand != -1:
        #     # -- first, we find a spot that sits RIGHT ON TOP of the surface...
        #     goal_pose[0] += sim.getObjectFloatParam(goal, sim.objfloatparam_objbbox_max_x)
        #     # ... then we will find a spot that considers the height of the object:
        #     goal_pose[0] += sim.getObjectFloatParam(obj_in_hand, sim.objfloatparam_objbbox_max_x) * (1.5 if target_object not in ["table", "worksurface"] else 1.25)

        #     # -- we also want to consider the orientation of the object
        #     orientation = sim.getObjectOrientation(goal, target)
        #     goal_pose[3:] = sim.buildPose(goal_pose[:3], [orientation[0], orientation[1], math.pi + rotate])[3:]

        # else:
        #     # -- account for fingertip placement on object:
        #     goal_pose[0] -= sim.getObjectFloatParam(goal, sim.objfloatparam_objbbox_max_x) * 3
        #     # -- try to match the orientation of the surface object:
        #     orientation = sim.getObjectOrientation(goal, robot)
        #     goal_pose[3:] = sim.buildPose(goal_pose[:3], [orientation[0], math.pi/2, orientation[2] + rotate])[3:]

        candidate_goal_poses.append(goal_pose)

    return candidate_goal_poses

sim.startSimulation()

try:
    for target_object in ["B_block_1", "D_block_3", "C_block_3"]:
        goal_poses = find_pose_for_ompl(
            robot_name=robot_name,
            target_object=target_object,
        )

        for _ in range(1):
            for G in goal_poses:
                success = ompl_path_planning(
                    target_object=target_object,
                    goal_pose=G,
                    ompl_args={
                        "ompl_state_resolution": float("5.0e-3"),
                        "ompl_use_lua": True,
                        "ompl_algorithm": "RRTStar",
                    },
                )
except Exception as e:
    print(e)
    pass

sim.stopSimulation()
pass
