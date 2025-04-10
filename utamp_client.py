import sys
import json
import argparse
import mysql.connector as sql

from typing import Type
from utils import Interfacer

# NOTE: you must have a file named "utamp.config.json" containing credentials to access the UTAMP server:

class UTAMPClient:
    def __init__(
        self,
        sim_interfacer: Type[Interfacer],
        config_fpath: str = 'utamp.config.json',
    ):
        try:
            self.cnx = sql.connect(**json.load(open(config_fpath, 'r')))

        except FileNotFoundError:
            print(f'[UTAMP-client] : No config file found at "{config_fpath}"!')
            sys.exit()

        self.cnx.autocommit = True
        self.cursor = self.cnx.cursor(buffered=True)

        self.sim_interfacer = sim_interfacer


    def parse_server_output(
        self,
        str_actions: str,
    ):
        # -- split the string based on semi-colons:
        str_actions = str_actions.split(':')
        time, traj = [], []
        gripper_action = None
        for t in range(3):
            coordinates = []
            for S in range(len(str_actions)):
                if f't_{t+1}' in str_actions[S]:
                    time.append( eval(str(str_actions[S]).split(',')[1]) )
                elif f'_{t+1}' in str_actions[S]:
                    coordinates.append( eval(str(str_actions[S]).split(',')[1]) )
                elif not gripper_action and 'gripper' in str(str_actions[S]):
                    gripper_action = eval(str_actions[S].split(',')[1][:-1])

            if coordinates:
                traj.append(coordinates)

        keypoints = {
            'time': time,
            'trajectory': traj,
        }

        return keypoints, gripper_action


    def encode_goal(
        self,
        goals_for_utamp: list[str],
    ) -> str:

        # -- parse through the goal predicates and format it as a string needed for UTAMP:
        obj_to_goals = {}

        for P in goals_for_utamp:
            # -- we want to get strings in the required format for the UTAMP system:
            pred_parts = P[1:-1].split(" ")
            if len(pred_parts) < 3:
                continue

            if bool(set(pred_parts[1:]).intersection(set(["hand", "table"]))):
                # -- ignore any references to the hand or the table -- this is being handled by UTAMP.
                continue

            if pred_parts[1] not in obj_to_goals:
                obj_to_goals[pred_parts[1]] = []

            # -- format all the predicates in the string format indicated by Alejandro:
            obj_to_goals[pred_parts[1]].append(f'{pred_parts[0]},{pred_parts[2]}')

        # -- post-process the goal states to identify any objects that should have "air" on it:
        for O in obj_to_goals:
            on_found = False
            for P in obj_to_goals[O]:
                if 'on,' in P:
                    on_found = True

            if not on_found:
                obj_to_goals[O].append('on,air')

        str_goal = str()
        for O in obj_to_goals:
            str_goal += f'{O}:{":".join(obj_to_goals[O])};'


    def perform_sensing(self) -> str:
        # -- get references to items from the Interfacer class to use here:
        sim_interfacer = self.sim_interfacer
        sim = sim_interfacer.sim

        # -- goal_objects :- list of all objects that are in the scene that are dynamic and respondable
        #       (does not consider the robot, the table, and other elements):
        goal_objects = sim_interfacer.objects_in_sim

        # -- name of the robot and its gripper:
        robot_name = sim_interfacer.robot_name
        robot_gripper = sim_interfacer.robot_gripper

        # -- get the robot's handle:
        robot_handle = sim.getObject(f'/{robot_name}')

        # -- list of all string components we will use to merge into a single sensing result string:
        str_objects = []

        # -- first, let's do sensing for the robot's hand, base, and the work surface:
        for x in [f'/{robot_name}', f'/{robot_name}/target', '/worksurface']:
            obj_handle = sim.getObject(x)
            obj_position, obj_orientation = sim.getObjectPosition(obj_handle, robot_handle), sim.getObjectOrientation(obj_handle, robot_handle)

            # -- getting bounding box dimensions of objects in the scene:
            if 'target' in x.lower():
                # NOTE: the motion planning should consider the dimensions of the robot's gripper:
                obj_handle = sim.getObject(f'/{robot_gripper}')
            elif robot_name in x:
                obj_handle = sim.getObject(f'/{robot_name}_link0_visual')

            obj_bb_min_x = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_min_x)
            obj_bb_max_x = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_max_x)
            obj_bb_min_y = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_min_y)
            obj_bb_max_y = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_max_y)
            obj_bb_min_z = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_min_z)
            obj_bb_max_z = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_max_z)

            if 'target' in x.lower():
                # NOTE: the motion planning only considers the height of each finger in the Panda's gripper:
                obj_handle = sim.getObject(f'/{robot_name}_leftfinger_visible')

            # NOTE: forcing the robot's bounding box to 0:
            if x == f'/{robot_name}':
                obj_bb_max_x = obj_bb_min_x = 0.0
                obj_bb_max_y = obj_bb_min_y = 0.0
                obj_bb_max_z = obj_bb_min_z = 0.0

            str_objects.append(
                f'{"hand" if "target" in x.lower() else "robot" if robot_name in x else "worksurface"}:'\
                f'x,{obj_position[0]}:'\
                f'y,{obj_position[1]}:'\
                f'z,{obj_position[2]}:'\
                f'roll,{obj_orientation[0]}:'\
                f'pitch,{obj_orientation[1]}:'\
                f'yaw,{obj_orientation[2]}:'\
                f'dx,{obj_bb_max_x-obj_bb_min_x}:'\
                f'dy,{obj_bb_max_y-obj_bb_min_y}:'\
                f'dz,{obj_bb_max_z-obj_bb_min_z};'
            )

        obj_index = 0
        while True:
            obj_handle = sim.getObjects(obj_index, sim.handle_all)
            if obj_handle == -1:
                break

            obj_name = sim.getObjectAlias(obj_handle)
            if obj_name in goal_objects:
                # print(obj_name)
                # -- get position and orientation of objects with respect to robot:
                obj_position = sim.getObjectPosition(obj_handle, robot_handle)
                obj_orientation = sim.getObjectOrientation(obj_handle, robot_handle)

                obj_bb_min_x = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_min_x)
                obj_bb_min_y = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_min_y)
                obj_bb_min_z = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_min_z)

                obj_bb_max_x = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_max_x)
                obj_bb_max_y = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_max_y)
                obj_bb_max_z = sim.getObjectFloatParam(obj_handle, sim.objfloatparam_objbbox_max_z)

                str_objects.append(
                    f'{obj_name}:'\
                    f'x,{obj_position[0]}:'\
                    f'y,{obj_position[1]}:'\
                    f'z,{obj_position[2]}:'\
                    f'roll,{obj_orientation[0]}:'\
                    f'pitch,{obj_orientation[1]}:'\
                    f'yaw,{obj_orientation[2]}:'\
                    f'dx,{obj_bb_max_x-obj_bb_min_x}:'\
                    f'dy,{obj_bb_max_y-obj_bb_min_y}:'\
                    f'dz,{obj_bb_max_z-obj_bb_min_z};'
                )

            obj_index += 1

        return "".join(str_objects)


    def plan_and_execute(
        self,
        goals_for_utamp: list = [],
        path_planning_method: int = 1, # 1 -- OMPL, not 1 -- spline
        verbose: bool = False,
    ) -> bool:

        cnx, cursor = self.cnx, self.cursor

        # ENCODE str_goal
        str_goal = self.encode_goal(goals_for_utamp)

        vars_to_insert = {
            'user_id': 51,
            'goal': str_goal,
            'objects': 'n/a',
            'actions': 'n/a',
            'status': 0,
        }

        sql_command = f'UPDATE utamp_data SET status = -1 WHERE user_id = {vars_to_insert["user_id"]}'
        cursor.execute(sql_command, vars_to_insert)
        cnx.commit()

        # UPDATE db.table SET goal = 'str_goal' WHERE id=USERID
        # UPDATE db.table SET status = 0 WHERE id=USERID (Goal provided)
        sql_command = f'UPDATE utamp_data SET status = 0, goal = "{vars_to_insert["goal"]}" WHERE user_id = {vars_to_insert["user_id"]}'
        cursor.execute(sql_command, vars_to_insert)
        cnx.commit()

        sql_command = ("SELECT * from utamp_data")
        cursor.execute(sql_command)

        if verbose:
            for user_id, goal, objects, actions, _, status, msg in cursor:
                print('user', user_id)
                print(goal)
                print(objects)
                print(actions)
                print(status)
                print(msg)
                print()

        # NOTE: status numbers:
        # Status	Meaning
        # -1	    Waiting for goal (updated by Server).
        #  0	    Goal/Sensing/Execution request concluded (updated by Client).
        #  1	    Execution request (updated by Server).
        #  2	    Execution in progress (updated by Client).
        #  3	    Execution failed (updated by Client).
        # 10	    Sensing request (updated by Server).
        # 20	    Sensing in progress (updated by Client).
        # 30	    Sensing failed (updated by Client).
        status = 0

        print(f"\n{'*' * 10} UTAMP CLIENT/SERVER INTERACTION {'*' * 10}\n")

        print(f" -- goals sent to utamp:\t{str_goal}\n")

        print('UTAMP interactions:')

        last_status = None

        while status > -1:
            sql_command = ("SELECT status, msg from utamp_data where user_id = 51")
            cursor.execute(sql_command)
            status, msg = cursor.fetchone()

            sql_command = ("SELECT status from utamp_data where user_id = 51")

            if last_status != (status, msg):
                print(f'status {status}:\t{msg}')
                last_status = (status, msg)

            if status == 1:
                # -- execution request:
                status = 2
                sql_command = f'UPDATE utamp_data SET status = {status} WHERE user_id = 51'
                cursor.execute(sql_command)
                # print('performing execution...')

                sql_command = ("SELECT actions from utamp_data where user_id = 51")
                cursor.execute(sql_command)
                str_actions = cursor.fetchone()[0]

                # -- we will parse the output given by the UTAMP server...
                keypoints, end_effector = self.parse_server_output(str_actions)
                # ... and then we will perform path planning with OMPL:
                self. sim_interfacer.execute_utamp(
                    goal_poses=keypoints['trajectory'][1:],
                    gripper_action=end_effector,
                    method=path_planning_method,
                )

                status = 0
                sql_command = f'UPDATE utamp_data SET status = {status} WHERE user_id = 51'
                cursor.execute(sql_command)

            if status == 10:
                # -- sensing request:
                status = 20

            if status == 20:
                sql_command = f'UPDATE utamp_data SET status = {status} WHERE user_id = 51'
                cursor.execute(sql_command)

                # -- use sensing function (uses the Interfacer object for certain references to the simulation environment):
                str_objects = self.perform_sensing()
                if verbose:
                    print(str_objects)

                if not str_objects:
                    # -- this means that sensing failed:
                    status = 30
                    sql_command = f'UPDATE utamp_data SET status = {status} WHERE user_id = 51'
                else:
                    # -- this means that sensing succeeded:
                    status = 0
                    sql_command = f'UPDATE utamp_data SET objects = "{str_objects}", status = {status} WHERE user_id = 51'

                cursor.execute(sql_command)

        if status == -1:
            return True

        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--goal",
        type=str, default="None",
        help="This specifies a set of goal predicates as a string."
    )

    parser.add_argument(
        "--scene",
        type=str,
        default='./',
        help="This specifies a set of goal predicates as a string."
    )

    args = parser.parse_args()
    if not eval(args.goal):
        utamp_goals = [
            "(on blockc blocka)",
            "(under blocka blockc)",
            "(on blocka air)",
        ]
    else:
        utamp_goals = eval(args.goal)

    utamp_client = UTAMPClient(
        sim_interfacer=Interfacer(
            scene_file_name=args.scene,
            robot_name="Panda",
            robot_gripper="Panda_gripper",
        ),
        config_fpath='utamp.config.json',
    )

    utamp_client.plan_and_execute(goals_for_utamp=utamp_goals)