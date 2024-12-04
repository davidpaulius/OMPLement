import os
import sys
import time
import math

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    print(' -- ERROR: Set up CoppeliaSim ZeroMQ client as described here: '\
          'https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm')
    sys.exit()

client = RemoteAPIClient(host='localhost')

sim = client.require('sim')
status = sim.loadScene(os.path.abspath('./panda_blocks_simple.ttt'))

# -- loading required modules for simulation:
simIK = client.require('simIK')
simOMPL = client.require('simOMPL')

def ompl_path_planning(
        goal_pose: list[float],
        robot_name: str = "Panda",
        num_ompl_attempts: int = 6,
        max_compute: int = 10,
        max_simplify: int = -1,
        len_path: int = 0,
    ) -> bool:

    # -- create a dummy object that will represent the target goal:
    target_goal = sim.createDummy(0.025)
    sim.setObjectPose(target_goal, goal_pose, sim.getObject(f'/{robot_name}'))
    sim.setObjectColor(target_goal, 0, sim.colorcomponent_ambient_diffuse, [0.0, 1.0, 1.0])
    sim.setObjectColor(target_goal, 0, sim.colorcomponent_emission, [0.6, 0.6, 0.6])
    sim.setObjectAlias(target_goal, 'OMPL_target')

    # -- we sleep for a bit so we can see this object appear in the sim:
    time.sleep(0.0001)

    ompl_script = sim.getScript(sim.scripttype_simulation, sim.getObject('/OMPLement'))

    path = sim.callScriptFunction(
        "ompl_path_planning",
        ompl_script,
        {
            "robot": robot_name,
            "goal": target_goal,
            "algorithm": simOMPL.Algorithm.RRTConnect,
            "num_attempts": num_ompl_attempts,
            "max_compute": max_compute,
            "max_simplify": max_simplify,
            "len_path": len_path,
        },
    )

    if path:
        sim.addLog(sim.getInt32Param(sim.intparam_verbosity), f'[OMPLement]: plan found!')

        # -- we need to disable the IK following done by the "target" dummy of the robot:
        # sim.setModelProperty(target, sim.modelproperty_scripts_inactive)
        ik_script = sim.getScript(sim.scripttype_simulation, sim.getObject(f"/{robot_name}"))
        if ik_script == -1:
            ik_script = sim.getScript(sim.scripttype_customization, sim.getObject(f"/{robot_name}"))

        sim.setObjectInt32Param(ik_script, sim.scriptintparam_enabled, 0)

        # -- with the computed path, we will gradually change the configuration of the robot:
        for P in range(len(path)):
            sim.callScriptFunction('setConfig', ompl_script, path[P])
            time.sleep(float('2.5e-3'))

        time.sleep(0.01)

        # -- we need to re-enable the IK following done by the "target" dummy of the robot:
        sim.setObjectPosition(sim.getObject(f'/{robot_name}/target'), sim.getObjectPosition(sim.getObject(f'/{robot_name}/tip')), -1)
        sim.setObjectOrientation(sim.getObject(f'/{robot_name}/target'), sim.getObjectOrientation(sim.getObject(f'/{robot_name}/tip')), -1)
        sim.setObjectInt32Param(ik_script, sim.scriptintparam_enabled, 1)

    else:
        sim.addLog(sim.getInt32Param(sim.intparam_verbosity), f'[OMPLement]: plan not found!')

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
    #     sim.addLog(sim.getInt32Param(sim.intparam_verbosity), f"table grounding: found empty spot: /{target_object}[{empty_spot['index']}]")
    #     if verbose:
    #         print(f"table grounding: found empty spot: /{target_object}[{empty_spot['index']}]")

    goal = sim.getObject(f"/{target_object}", {"index": index})

    candidate_goal_poses = []

    for rotate in [0.0, (math.pi/2), (math.pi), (math.pi*2)]:
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
            goal_pose[2] += sim.getObjectFloatParam(goal, sim.objfloatparam_objbbox_max_z) * 1.5
            # -- try to match the orientation of the surface object:
            orientation = sim.getObjectOrientation(goal, robot)
            goal_pose[3:] = sim.buildPose(goal_pose[:3], [-(math.pi), orientation[1], orientation[2] + rotate])[3:]


        candidate_goal_poses.append(goal_pose)

    return candidate_goal_poses

sim.startSimulation()

for target_object in ["D_block_3", "C_block_3", "B_block_1"]:
    goal_poses = find_pose_for_ompl(
        robot_name="Panda",
        target_object=target_object)

    success = ompl_path_planning(
        goal_pose=goal_poses[0],
    )

sim.stopSimulation()
