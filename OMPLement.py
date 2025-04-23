'''luaExec

function getConfig_lua()
    -- get the current joint configuration
    local retVal={}
    for J=1,#joint_handles,1 do
        retVal[J]=sim.getJointPosition(joint_handles[J])
    end
    return retVal
end


function setConfig_lua(config)
    -- apply a joint configuration to the robot
    for J=1,#joint_handles,1 do
        sim.setJointPosition(joint_handles[J],config[J])
    end
end


function stateValidationCollision_lua(config)
    -- check if a configuration is valid, i.e., doesn't collide:
    -- save current config:
    local tmp = getConfig_lua()

    -- apply new config:
    setConfig_lua(config)

    -- does new config collide?
    local is_collision, _ = sim.checkCollision(robot_collection,sim.handle_all)

    -- restore original config:
    setConfig_lua(tmp)

    -- make sure to return true/True when a state is valid:
    return (is_collision == 0)
end


function luaStateValidationData(data)
    -- 'robot' and 'tip' refer to the handles of the robot and the tip of the end-effector:
    robot = data["robot"]
    tip = data["tip"]

    -- 'constrained_axis' refers to any value in ["free", "x", "y", "z"]:
    axes = data["axes"]

    -- 'sim_threshold' will reflect how "aligned" the poses must be along a given axis;
    --     for *dot product similarity* -- if two vectors are aligned, their dot product will be equal to 1:
    sim_threshold = data["alignment"]

    pose_limits = nil
    if data["pose_limits"] then
        pose_limits = data["pose_limits"]
    end

    -- TODO: are there any other values needed?
end


function stateValidationFixedAxis_lua(config)
    -- save current config:
    local tmp = getConfig_lua()

    -- get the pre-motion pose of the end-effector's tip:
    local initial_pose = sim.getObjectPose(tip, robot)

    -- apply new config:
    setConfig_lua(config)

    -- get the new pose of the end-effector's tip after moving to the candidate config:
    local next_pose = sim.getObjectPose(tip, robot)

    -- does new config collide?
    local is_collision, _ = sim.checkCollision(robot_collection, sim.handle_all)

    -- restore original config:
    setConfig_lua(tmp)

    if (is_collision ~= 0) then
        return false
    end

    -- NOTE: the initial pose of the robot's hand will be passed by auxiliary function:
    --  variable names: axis, margin, initial_pose
    local is_aligned = checkAxisSimilarity_lua(initial_pose, next_pose)

    -- if we have passed both checks, then this is a valid state:
    return (is_aligned == true)
end


function stateValidationPoseLimits_lua(config)
     -- save current config:
    local tmp = getConfig_lua()

    -- apply new config:
    setConfig_lua(config)

    -- get the new pose of the end-effector's tip after moving to the candidate config:
    local next_pose = sim.getObjectPose(tip, robot)

    -- does new config collide?
    local is_collision, _ = sim.checkCollision(robot_collection, sim.handle_all)

    -- restore original config:
    setConfig_lua(tmp)

    -- we will focus on the orientation portion (quaternions) of the pose:
    for X=4,#next_pose,1 do
        if (next_pose[X] > pose_limits["max"][X]) then
            return false
        elseif (next_pose[X] < pose_limits["min"][X]) then
            return false
        end
    end

    -- if we have passed both checks, then this is a valid state:
    return (is_collision == 0)
end


function luaFindIKConfig(data)
    -- get necessary handles for the state validation portion:
    local ikEnv = data["ikEnv"]
    local ikJointHandles = data["ikJointHandles"]
    local ikGroup = data["ikGroup"]

    robot_collection = data["robot_collection"]
    joint_handles = data["joint_handles"]

    params = {
        maxDist = 0.05,
        maxTime = 1,
        findMultiple = false, -- change to True to find multiple solutions
        pMetric = {0.05,0.05,0.05,0.1},
        cb = stateValidationCollision_lua
    }

    local configs = simIK.findConfigs(ikEnv, ikGroup, ikJointHandles, params)

    return configs

end


function luaOMPLCompute(data)
    -- NOTE: you need the following variables to be sent as a dictionary/table:
    if not data["ompl_task"] then
        sim.addLog(sim.verbosity_errors, "ERROR: no 'ompl_task' key-value pair provided!")
        sim.stopSimulation()
    end

    local ompl_task = data["ompl_task"]
    local ompl_max_compute = data["ompl_max_compute"]
    local ompl_max_simplify = data["ompl_max_simplify"]
    local ompl_len_path = data["ompl_len_path"]

    -- run the OMPL function from lua:
    result, path = simOMPL.compute(ompl_task, ompl_max_compute, ompl_max_simplify, ompl_len_path)

    return {ompl_task, result, path}
end


---------------------------------------------------------------------------------------
-- NOTE: this function is a modified version of a function from the following thread:
--      https://forum.coppeliarobotics.com/viewtopic.php?t=10200
---------------------------------------------------------------------------------------
function checkAxisSimilarity_lua(poseA, poseB)

    -- NOTE: axis can be some kind of string in ["x", "y", "z", "free"],
    --  'sim_threshold' refers to how "similar" the axis must be aligned to some given reference:
    local xAxis = {1, 0, 0}  -- X-axis vector (global frame) for object A
    local yAxis = {0, 1, 0}  -- Y-axis vector (global frame) for object A
    local zAxis = {0, 0, 1}  -- Z-axis vector (global frame) for object A

    -- extract the orientation components from the 7D pose
    --  and make rotation matrices that simply have direction:
    local vectorA = {0, 0, 0, poseA[4], poseA[5], poseA[6], poseA[7]}
    local vectorB = {0, 0, 0, poseB[4], poseB[5], poseB[6], poseB[7]}

    -- Convert poses to to rotation matrices:
    local rotMatrixA = sim.poseToMatrix(vectorA)
    local rotMatrixB = sim.poseToMatrix(vectorB)

    local transformedAxisA, transformedAxisA = -1, -1

    for _, axis in ipairs(axes) do
        if axis == "x" then
            -- Extract X-axis from the rotation matrices
            transformedAxisA = sim.multiplyVector(rotMatrixA, xAxis) -- gets first column of rotation matrix - represents vector of X axis {Xx, Xy, Xz}
            transformedAxisB = sim.multiplyVector(rotMatrixB, xAxis)
        elseif axis == "y" then
            -- Extract Y-axis from the rotation matrices
            transformedAxisA = sim.multiplyVector(rotMatrixA, yAxis)
            transformedAxisB = sim.multiplyVector(rotMatrixB, yAxis)
        elseif axis == "z" then
            -- Extract Y-axis from the rotation matrices
            transformedAxisA = sim.multiplyVector(rotMatrixA, zAxis)
            transformedAxisB = sim.multiplyVector(rotMatrixB, zAxis)
        elseif axis == "free" then
            -- just return true, as we don't need to care about fixed orientations:
            return true
        end
    end

    -- Check similarity between the X-axes: dot product of a vector with itself is the square of its magnitude
    local dotProduct = (transformedAxisA[1] * transformedAxisB[1])
        + (transformedAxisA[2] * transformedAxisB[2])
        + (transformedAxisA[3] * transformedAxisB[3])

    if dotProduct < sim_threshold then -- if dotProduct is near to 1 then X axes are similarly alligned.
        return false
    end

    return true
end


'''


def sysCall_init():
    sim = require('sim')
    simIK = require('simIK')
    simOMPL = require('simOMPL')
    math = require('math')

    sim.addLog(sim.verbosity_default, "[OMPLement] : Loading OMPL motion planning script...")

    self.ompl_use_lua = True
    self.ompl_use_state_validation = True
    self.verbose = True

    if self.ompl_use_lua:
        sim.addLog(sim.verbosity_default, "[OMPLement] : Using Lua-based OMPL functions! (ompl_use_lua=True)")


def visualizePath(path, rgb):
    _lineContainer=sim.addDrawingObject(sim.drawing_lines,3,0,-1,99999,rgb)
    sim.addDrawingObjectItem(_lineContainer,None)
    if path:
        #lb=sim.setStepping(True)
        initConfig=getConfig_python()
        for i in range(1, len(path)):
            config1, config2 = path[i-1], path[i]
            setConfig_python(config1)
            lineDat=sim.getObjectPosition(self.tip)
            setConfig_python(config2)
            lineDat[3:]=sim.getObjectPosition(self.tip)
            sim.addDrawingObjectItem(_lineContainer,lineDat)

        setConfig_python(initConfig)
    return _lineContainer


def sysCall_thread():
    pass

def sysCall_addOnScriptSuspend():
    pass


def sysCall_cleanup():
    sim.addLog(sim.verbosity_default, "[OMPLement] : Unloading OMPL motion planning script...")


def getConfig_python():
    config = [-1] * len(self.joint_handles)
    for J in range(len(self.joint_handles)):
        config[J] = sim.getJointPosition(self.joint_handles[J])
    return config


def setConfig_python(config):
    for J in range(len(self.joint_handles)):
        sim.setJointPosition(self.joint_handles[J], config[J])


def stateValidationCollision_python(config):
    # -- check if a configuration is valid, i.e. doesn't collide
    # -- save current config:
    tmp = getConfig_python()

    # -- apply new config:
    setConfig_python(config)

    # -- does new config collide?
    objs_in_collision = []
    is_collision, handles = sim.checkCollision(self.robot_collection,sim.handle_all)

    if is_collision == 1 and handles[1] not in objs_in_collision:
        objs_in_collision.append(sim.getObjectAlias(handles[1]))

    #if self.verbose and bool(objs_in_collision):
    #    sim.addLog(sim.verbosity_scriptwarnings, f"collision found with: {objs_in_collision}")

    # -- restore original config:
    setConfig_python(tmp)

    return (is_collision == 0)


def stateValidationOrientation_python(config):
    if not stateValidationCollision_python(config):
        return False


def find_ik_config(args):
    # -- parse all the arguments sent to this function and make them global variables (i.e., "self.XXX"):
    if not self.robot or not self.robot_collection:
        parse_args(args)

    # -- Prepare robot collection:
    self.robot_collection = sim.createCollection()
    sim.addItemToCollection(self.robot_collection, sim.handle_tree, self.robot, 0)

    collection_objs = [(x, sim.getObjectAlias(x)) for x in sim.getCollectionObjects(self.robot_collection)]

    # -- prepare an ik task (in order to be able to find configs that match specific end-effector poses):
    ikEnv = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)
    ikElement, simToIkObjectMapping, _ = simIK.addElementFromScene(ikEnv,ikGroup,self.robot,self.tip,self.goal,simIK.constraint_pose)
    simIK.syncFromSim(ikEnv, [ikGroup])

    # -- get a few handles from the IK world:
    ikJointHandles = []
    for J in range(len(self.joint_handles)):
        ikJointHandles.append(simToIkObjectMapping[self.joint_handles[J]])

    ikGoal=simToIkObjectMapping[self.goal]
    ikBase=simToIkObjectMapping[self.robot]
    ikTip=simToIkObjectMapping[self.tip]

    pose = sim.getObjectPose(self.goal, self.robot)

    simIK.setObjectPose(ikEnv,ikGoal,ikBase,pose)

    if self.ompl_use_lua:
        # -- run simIK.findConfigs from the Lua side:
        configs = sim.callScriptFunction(
            "luaFindIKConfig",
            sim.handle_self,
            {
                "ikEnv": ikEnv,
                "ikGroup": ikGroup,
                "ikJointHandles": ikJointHandles,
                "robot_collection": self.robot_collection,
                "joint_handles": self.joint_handles,
            },
        )

    else:
        # -- run simIK.findConfigs from the Python side:
        params = {
            'maxDist': 0.05,
            'maxTime': 3,
            'findMultiple': False, # -- change to True to find multiple solutions
            'pMetric': [0.05,0.05,0.05,0.1],
            'cb': stateValidationCollision_python,
        }
        configs = simIK.findConfigs(ikEnv, ikGroup, ikJointHandles, params)

    # NOTE: check here for more info on how a valid configuration is found via IK:
    #   https://manual.coppeliarobotics.com/en/simIK.htm#simIK.findConfigs
    if bool(configs):
        # -- found a robot config that matches the desired pose!
        return configs[0]

    return None
#end


def parse_args(args):
    # -- first check if the robot name and goal handle have been provided to the function:
    assert "robot" in args, "[OMPLement] : Robot name not defined!"
    robot_name = args["robot"]

    self.robot = sim.getObject(f'/{robot_name}')

    # NOTE: we will create a dummy object representing the target for planning!
    # -- extract the goal object given as input to this function:
    assert "goal" in args, "[OMPLement] : Goal not defined!"
    self.goal = args["goal"]

    # -- arm_prefix :- you can define the name format for joints (in the case where maybe there is a particular
    #   set of joints for which you want to do motion planning -- e.g., Spot robot has arm joints separate to legs)
    if "arm_prefix" in args:
        joint_prefix = f"/{args['arm_prefix']}"
    else:
        joint_prefix = f"/{robot_name}/joint"
    self.tip = sim.getObject(f'/{robot_name}/tip')

    assert self.robot != -1, "[OMPLement] : Robot base not defined!"
    assert self.tip != -1, "[OMPLement] : End-effector tip not defined!"

    ################################################################################################

    # NOTE: you need to know how many joints the robot you're using has;
    #   ideally, these joints should have some naming convention like in the loop below:
    self.joint_handles = []

    num_joints = 1
    while True:
        # -- using "noError" so default handle is -1 (if not found);
        #    read more here: https://manual.coppeliarobotics.com/en/regularApi/simGetObject.htm
        obj_handle = sim.getObject(f'{joint_prefix}', {"noError": True, "index":(num_joints-1)})

        if obj_handle == -1: break

        self.joint_handles.append(obj_handle)
        num_joints += 1

    # -- we will only use the first three joints (3) for projections:
    self.joint_projections = list([1] * len(self.joint_handles))

    sim.addLog(sim.verbosity_default, f'[OMPLement] : Number of joints for robot "{robot_name}" - {len(self.joint_handles)}')

    # -- first check if the robot name and goal handle have been provided to the function:
    assert "robot" in args, "[OMPLement] : Robot name not defined!"
    self.robot_name = args["robot"]

    # -- make the robot collection a global object:
    self.robot_collection = sim.createCollection()
    sim.addItemToCollection(self.robot_collection, sim.handle_tree, self.robot, 0)

    # NOTE: we will create a dummy object representing the target for planning!
    # -- extract the goal object given as input to this function:
    assert "goal" in args, "[OMPLement] : Goal not defined!"

    self.ompl_algorithm = simOMPL.Algorithm.RRTConnect
    if "ompl_algorithm" in args:
        self.ompl_algorithm = args["ompl_algorithm"]

    self.ompl_max_compute = 15
    if "ompl_max_compute" in args:
        self.ompl_max_compute = args["ompl_max_compute"]

    self.ompl_max_simplify = -1
    if "ompl_max_simplify" in args:
        self.ompl_max_simplify = args["ompl_max_simplify"]

    # -- ompl_len_path :- number of states for the path (default: 0 -- we leave it to OMPL)
    self.ompl_len_path = 0
    if "ompl_len_path" in args:
        self.ompl_len_path = args["ompl_len_path"]

    # -- ompl_num_attempts :- we have this functionality because simOMPL.compute() can reuse previously computed data
    #   Source: https://manual.coppeliarobotics.com/en/pathAndMotionPlanningModules.htm

    # -- check if the number of attempts for OMPL to solve a problem has been defined:
    self.ompl_num_attempts = 5
    if "ompl_num_attempts" in args:
        self.ompl_num_attempts = args["ompl_num_attempts"]

    # -- ompl_state_resolution :- this is a value that specifies the resolution for OMPL to find a solution:
    if "ompl_state_resolution" in args:
        self.ompl_state_resolution = args["ompl_state_resolution"]
    else:
        # -- by default, use this resolution value:
        self.ompl_state_resolution = float("5.0e-3")

    # NOTE: we should always use lua whenever possible since it is faster:
    if "ompl_use_lua" in args:
        self.ompl_use_lua = args["ompl_use_lua"]

    # -- ompl_use_state_validation :- this indicates whether state validation will be used in OMPL computation:
    self.ompl_use_state_validation = True
    if "ompl_use_state_validation" in args:
        self.ompl_use_state_validation = args["ompl_use_state_validation"]

    # -- ompl_motion_constraint :- this indicates whether some axis needs to be fixed for path planning:
    self.ompl_motion_constraint = "free"
    if "ompl_motion_constraint" in args:
        self.ompl_motion_constraint = args["ompl_motion_constraint"]

    # NOTE: rather than restricting motion based on a single axis as above,
    #   we can indicate limits for each axis:
    self.ompl_pose_limits = None
    if "ompl_pose_limits" in args:
        self.ompl_pose_limits = args["ompl_pose_limits"]

#end


def execute_trajectory_configs(data):
    sim.setStepping(True)
    sim.step()

    """
    vel = 110
    accel = 40
    jerk = 80
    maxVel = [vel*math.pi/180, vel*math.pi/180, vel*math.pi/180, vel*math.pi/180, vel*math.pi/180, vel*math.pi/180, vel*math.pi/180]
    maxAccel = [accel*math.pi/180, accel*math.pi/180, accel*math.pi/180, accel*math.pi/180, accel*math.pi/180, accel*math.pi/180, accel*math.pi/180]
    maxJerk = [jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180, jerk*math.pi/180]
    """

    for P in range(len(data["path"])):
        params = {
            'joints': self.joint_handles,
            'targetPos': path[P],
            'maxVel': data["maxVel"],
            'maxAccel': data["maxAccel"],
            'maxJerk': data["maxJerk"],
        }
        sim.moveToConfig(params)
        sim.step()

    sim.setStepping(False)
    return True
#end


def ompl_path_planning(args):
    """
    This function requires a dictionary containing the following fields:
        1. "robot" :- the name of the robot's base in the scenario
        2. "goal" :- the object handle for a target (this should be some kind of dummy object -- refer to Python code for example)
        3. "ompl_algorithm" :- the name of the motion planning algorithm to use (by default, "RRTstar" will be used)
        4. "ompl_num_attempts" :- the number of times to run OMPL (default: 20)
        5. "ompl_max_compute" :- the maximum time (in seconds) allotted to computing a solution
        6. "ompl_max_simplify" :- the maximum time (in seconds) allotted to simplifying a solution
        7. "ompl_len_path" :- the number of states for path generation (default: leave it to OMPL)
    """

    # -- parse all the arguments sent to this function and make them global variables (i.e., "self.XXX"):
    parse_args(args)

    # -- find a valid configuration that puts the robot's gripper at the goal location:
    valid_config = find_ik_config(args)

    ################################################################################################

    # NOTE: the path contains a Mx1 vector, which needs to be transformed to NxJ vector, where N = M/J.
    # -- the final path will be stored as a NxJ matrix, where N = number of points in trajectory and J = number of joints.
    final_path = []

    # NOTE: the total distance gives us some kind of metric about how long the path is, which may be useful for interpolation:
    total_distance = -1
    start_to_end = -1

    if bool(valid_config):
        # -- found a robot config that matches the desired pose!
        sim.addLog(sim.verbosity_scriptwarnings, "[OMPLement] : valid configuration found!")

        self.joint_weights = [1.0] * len(self.joint_projections)

        # -- Now find a collision-free path (via path planning) that brings us from current config to the found config:
        self.ompl_task = simOMPL.createTask('ompl_task')
        simOMPL.setAlgorithm(self.ompl_task, self.ompl_algorithm)
        simOMPL.setStateSpaceForJoints(self.ompl_task, self.joint_handles, self.joint_projections, self.joint_weights)
        simOMPL.setCollisionPairs(self.ompl_task,[self.robot_collection, sim.handle_all])
        simOMPL.setStartState(self.ompl_task,getConfig_python())
        simOMPL.setGoalState(self.ompl_task,valid_config)
        simOMPL.setStateValidityCheckingResolution(self.ompl_task, self.ompl_state_resolution)
        simOMPL.setVerboseLevel(self.ompl_task, 1)

        if self.ompl_use_state_validation:
            # NOTE: state validation is typically slow, but pretty decent when used in lua mode:
            if self.ompl_motion_constraint == "free":
                if self.verbose:
                    sim.addLog(sim.verbosity_default, "[OMPLement] : Considering free-axis orientation...")
                if not self.ompl_use_lua:
                    simOMPL.setStateValidationCallback(self.ompl_task, stateValidationCollision_python)
                else:
                    simOMPL.setStateValidationCallback(self.ompl_task, 'stateValidationCollision_lua')

            else:
                # -- we are adding extra constraints for state validation (beyond collision):
                state_validation_data = {
                    "robot": self.robot,
                    "tip": self.tip,
                    "alignment": 0.985,
                    "pose_limits": self.ompl_pose_limits,
                }

                # -- send some necessary values to the state validation checker:
                sim.callScriptFunction("luaStateValidationData", sim.handle_self, state_validation_data)

                if self.ompl_pose_limits:
                    if self.verbose:
                        sim.addLog(sim.verbosity_default, "[OMPLement] : Using min-max pose constraints...")

                    # -- we are going to use the lua version no matter what:
                    simOMPL.setStateValidationCallback(self.ompl_task, "stateValidationPoseLimits_lua")

                else:
                    # -- we will split the string into characters which can be "x", "y", or "z"
                    axes = [axis for axis in str(self.ompl_motion_constraint)]

                    for A in axes:
                        # -- if we are given any invalid axes, we will just default to "free" constraints:
                        if A not in ["x", "y", "z"]:
                            axes = ["free"]
                            break

                    state_validation_data["axes"] = axes

                    # -- update the data sent to the state validation callback side:
                    sim.callScriptFunction("luaStateValidationData", sim.handle_self, state_validation_data)

                    if self.verbose:
                        sim.addLog(sim.verbosity_default, f"[OMPLement] : Considering fixed-axis orientation (axis={self.ompl_motion_constraint})...")

                    # -- we are going to use the lua version no matter what:
                    simOMPL.setStateValidationCallback(self.ompl_task, "stateValidationFixedAxis_lua")

        simOMPL.setup(self.ompl_task)
        simOMPL.printTaskInfo(self.ompl_task)

        for _ in range(self.ompl_num_attempts):
            # -- read more about compute operation here: https://manual.coppeliarobotics.com/en/simOMPL.htm#compute
            if self.ompl_use_lua:
                data = {
                    "ompl_task": self.ompl_task,
                    "ompl_max_compute": self.ompl_max_compute,
                    "ompl_max_simplify": self.ompl_max_simplify,
                    "ompl_len_path": self.ompl_len_path,
                    "ompl_use_state_validation": self.ompl_use_state_validation,
                }
                output = sim.callScriptFunction("luaOMPLCompute", sim.handle_self, data)
                self.ompl_task, result, path = output[0], output[1], output[2]
            else:
                result, path = simOMPL.compute(
                    self.ompl_task,
                    self.ompl_max_compute,
                    self.ompl_max_simplify,
                    self.ompl_len_path,
                )

            # -- we will see if there was an exact solution found;
            #    that way we know if we might need to loop back around again to find the solution
            is_exact_solution = simOMPL.hasExactSolution(self.ompl_task)

            # -- if no exact solution was found... then maybe we will compute again?
            sim.addLog(sim.verbosity_default, f"[OMPLement] : Exact solution found? -- {is_exact_solution}")

            # sim.addLog(sim.verbosity_default, f"[OMPLement] : Distance to target -- {simOMPL.getGoalDistance(self.ompl_task)}")

            if result and is_exact_solution:
                # -- We found a collision-free path!

                # NOTE: the path contains a Mx1 vector, which needs to be transformed to NxJ vector, where N = M/J.
                # -- the final path will be stored as a NxJ matrix, where N = number of points in trajectory and J = number of joints.
                final_path = []
                for x in range(0, len(path), len(self.joint_handles)):
                    final_path.append(path[x:x+len(self.joint_handles)])

                # -- also compute the total length of the computed path:
                total_distance = 0

                # -- save original configuration:
                tmp = getConfig_python()

                for x in range(len(final_path) - 1):
                    # -- set the joints to configuration x:
                    setConfig_python(final_path[x])
                    config_1 = sim.getObjectPose(self.tip, sim.handle_world)

                    # -- set the joints to configuration (x+1):
                    setConfig_python(final_path[x+1])
                    config_2 = sim.getObjectPose(self.tip, sim.handle_world)

                    total_distance += sim.getConfigDistance(config_1, config_2)

                # -- reset back to original configuration:
                setConfig_python(tmp)

                # -- now let's do the total distance between first and last state:
                ini_config = sim.getObjectPose(self.tip, sim.handle_world)
                setConfig_python(final_path[-1])
                end_config = sim.getObjectPose(self.tip, sim.handle_world)

                setConfig_python(tmp)

                start_to_end = sim.getConfigDistance(ini_config, end_config)

                sim.addLog(sim.verbosity_default, f"[OMPLement] : Length of path: {int(simOMPL.getPathStateCount(self.ompl_task,path))}")
                sim.addLog(sim.verbosity_default, f"[OMPLement] :  -- total distance travelled by path:\t{total_distance}")
                sim.addLog(sim.verbosity_default, f"[OMPLement] :  -- ini to end configuration distance:\t{start_to_end}")

                assert simOMPL.getPathStateCount(self.ompl_task,path) == len(final_path), "[OMPLement] : error in path rebuild?"

                break


        simOMPL.destroyTask(self.ompl_task)

    else:
        sim.addLog(sim.verbosity_scriptwarnings, "[OMPLement] : no configuration found!")

    return {
        "path": final_path,
        "total_distance": total_distance,
        "ini_to_end_distance": start_to_end,
    }

#end