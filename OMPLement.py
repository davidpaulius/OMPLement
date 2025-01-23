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

    return (is_collision > 0)
end


function luaFindIKConfig(data)
    -- get necessary handles for the state validation portion:
    local ikEnv = data["ikEnv"]
    local ikJointHandles = data["ikJointHandles"]
    local ikGroup = data["ikGroup"]

    robot_collection = data["robot_collection"]
    joint_handles = data["joint_handles"]

    params = {
        maxDist = 0.1,
        maxTime = 10,
        findMultiple = false, -- change to True to find multiple solutions
        pMetric = {0.05,0.05,0.05,0.1},
        --cb = 'stateValidationCollision_lua'
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
    local max_compute = data["max_compute"]
    local max_simplify = data["max_simplify"]
    local len_path = data["len_path"]

    if data["use_state_validation"] and data["use_state_validation"] == true then
        simOMPL.setStateValidationCallback(ompl_task, 'stateValidationCollision_lua')
    end

    -- run the OMPL function from lua:
    result, path = simOMPL.compute(ompl_task, max_compute, max_simplify, len_path)

    return {ompl_task, result, path}
end


-- Function to check if the X-axis of object B is similar to the X-axis of object A
function checkXAxisSimilarity(orientationA, orientationB, margin)
    local xAxisA = {1, 0, 0}  -- X-axis vector (global frame) for object A

    -- Convert orientations to rotation matrices
    local rotMatrixA = sim.buildMatrix({0, 0, 0}, orientationA)
    local rotMatrixB = sim.buildMatrix({0, 0, 0}, orientationB)

    -- Extract X-axis from the rotation matrices
    local transformedXAxisA = sim.multiplyVector(rotMatrixA, xAxisA) -- gets first column of rotation matrix - represents vector of X axis {Xx, Xy, Xz}
    local transformedXAxisB = sim.multiplyVector(rotMatrixB, xAxisA)

    -- Check similarity between the X-axes: dot product of a vector with itself is the square of its magnitude
    local dotProduct = transformedXAxisA[1] * transformedXAxisB[1] + transformedXAxisA[2] * transformedXAxisB[2] + transformedXAxisA[3] * transformedXAxisB[3]

    if dotProduct > margin then -- if dotProduct is near to 1 then X axes are similarly alligned.
        return true
    end
    return false
end


function checkAxisSimilarity(orientationA, orientationB, axis, margin)
    local xAxisA = {1, 0, 0}  -- X-axis vector (global frame) for object A
    local yAxisA = {0, 1, 0}  -- Y-axis vector (global frame) for object A
    local zAxisA = {0, 0, 1}  -- Z-axis vector (global frame) for object A

    -- Convert orientations to rotation matrices
    local rotMatrixA = sim.buildMatrix({0, 0, 0}, orientationA)
    local rotMatrixB = sim.buildMatrix({0, 0, 0}, orientationB)

    if axis == "x" then
        -- Extract X-axis from the rotation matrices
        local transformedAxisA = sim.multiplyVector(rotMatrixA, xAxisA) -- gets first column of rotation matrix - represents vector of X axis {Xx, Xy, Xz}
        local transformedAxisB = sim.multiplyVector(rotMatrixB, xAxisA)
    elseif axis == "y" then
        -- Extract Y-axis from the rotation matrices
        local transformedAxisA = sim.multiplyVector(rotMatrixA, yAxisA)
        local transformedAxisB = sim.multiplyVector(rotMatrixB, yAxisA)
    elseif axis == "z" then
        -- Extract Y-axis from the rotation matrices
        local transformedAxisA = sim.multiplyVector(rotMatrixA, zAxisA)
        local transformedAxisB = sim.multiplyVector(rotMatrixB, zAxisA)
    elseif axis == "free" then
        -- just return true, as we don't need to care about fixed orientations:
        return true
    end

    -- Check similarity between the X-axes: dot product of a vector with itself is the square of its magnitude
    local dotProduct = transformedAxisA[1] * transformedAxisB[1] + transformedAxisA[2] * transformedAxisB[2] + transformedAxisA[3] * transformedAxisB[3]

    if dotProduct > margin then -- if dotProduct is near to 1 then X axes are similarly alligned.
        return true
    end
    return false
end


'''

def sysCall_init():
    sim = require('sim')
    simIK = require('simIK')
    simOMPL = require('simOMPL')
    math = require('math')

    sim.addLog(sim.verbosity_default, "[OMPLement] : Loading OMPL motion planning script...")

    self.use_lua = True
    self.verbose = True
    self.use_state_validation = True

    if self.use_lua:
        sim.addLog(sim.verbosity_default, "[OMPLement] : Using Lua-based OMPL functions! (use_lua=True)")


def visualizePath(path, rgb):
    _lineContainer=sim.addDrawingObject(sim.drawing_lines,3,0,-1,99999,rgb)
    sim.addDrawingObjectItem(_lineContainer,None)
    if path:
        #lb=sim.setStepping(True)
        initConfig=getConfig()
        for i in range(1, len(path)):
            config1, config2 = path[i-1], path[i]
            setConfig(config1)
            lineDat=sim.getObjectPosition(self.tip)
            setConfig(config2)
            lineDat[3:]=sim.getObjectPosition(self.tip)
            sim.addDrawingObjectItem(_lineContainer,lineDat)

        setConfig(initConfig)
    return _lineContainer


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

    # -- restore original config:
    setConfig_python(tmp)

    if self.verbose and bool(objs_in_collision):
        sim.addLog(sim.verbosity_scriptwarnings, f"collision found with: {objs_in_collision}")

    return not bool(is_collision)


def stateValidationOrientation_python(config):
    if not stateValidationCollision_python(config):
        return False

    #return checkAxisSimilarity

"""
def checkAxisSimilarity(orientationA, orientationB, axis, margin):
    local xAxisA = {1, 0, 0}  -- X-axis vector (global frame) for object A
    local yAxisA = {0, 1, 0}  -- Y-axis vector (global frame) for object A
    local zAxisA = {0, 0, 1}  -- Z-axis vector (global frame) for object A

    -- Convert orientations to rotation matrices
    local rotMatrixA = sim.buildMatrix({0, 0, 0}, orientationA)
    local rotMatrixB = sim.buildMatrix({0, 0, 0}, orientationB)

    if axis == "x" then
        -- Extract X-axis from the rotation matrices
        local transformedAxisA = sim.multiplyVector(rotMatrixA, xAxisA) -- gets first column of rotation matrix - represents vector of X axis {Xx, Xy, Xz}
        local transformedAxisB = sim.multiplyVector(rotMatrixB, xAxisA)
    elseif axis == "y" then
        -- Extract Y-axis from the rotation matrices
        local transformedAxisA = sim.multiplyVector(rotMatrixA, yAxisA)
        local transformedAxisB = sim.multiplyVector(rotMatrixB, yAxisA)
    elseif axis == "z" then
        -- Extract Y-axis from the rotation matrices
        local transformedAxisA = sim.multiplyVector(rotMatrixA, zAxisA)
        local transformedAxisB = sim.multiplyVector(rotMatrixB, zAxisA)
    elseif axis == "free" then
        -- just return true, as we don't need to care about fixed orientations:
        return true
    end

    -- Check similarity between the X-axes: dot product of a vector with itself is the square of its magnitude
    local dotProduct = transformedAxisA[1] * transformedAxisB[1] + transformedAxisA[2] * transformedAxisB[2] + transformedAxisA[3] * transformedAxisB[3]

    if dotProduct > margin then -- if dotProduct is near to 1 then X axes are similarly alligned.
        return true
    end
    return false
end
"""

def findIKConfig(args):
    # -- Prepare robot collection:
    self.robot_collection = sim.createCollection()
    sim.addItemToCollection(self.robot_collection, sim.handle_tree, self.robot, 0)

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

    if self.use_lua:
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
            'maxDist': 0.1,
            'maxTime': 10,
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

    #self.pose = sim.getObjectPose(self.goal, self.robot)

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

    # NOTE: we will create a dummy object representing the target for planning!
    # -- extract the goal object given as input to this function:
    assert "goal" in args, "[OMPLement] : Goal not defined!"

    self.ompl_algorithm = simOMPL.Algorithm.RRTConnect
    if "algorithm" in args: self.ompl_algorithm = args["algorithm"]

    self.max_compute = 5
    if "max_compute" in args: self.max_compute = args["max_compute"]

    self.max_simplify = -1
    if "max_simplify" in args: self.max_simplify = args["max_simplify"]

    # -- len_path :- number of states for the path (default: 0 -- we leave it to OMPL)
    self.len_path = 0
    if "len_path" in args: self.len_path = args["len_path"]

    # -- num_ompl_attempts :- we have this functionality because simOMPL.compute() can reuse previously computed data
    #   Source: https://manual.coppeliarobotics.com/en/pathAndMotionPlanningModules.htm
    # -- check if the number of attempts for OMPL to solve a problem has been defined:
    if "num_attempts" in args:
        self.num_ompl_attempts = args["num_attempts"]
    else:
        self.num_ompl_attempts = 5

    # -- state_resolution :- this is a value that specifies the resolution for OMPL to find a solution:
    if "state_resolution" in args:
        self.state_resolution = args["state_resolution"]
    else:
        # -- by default, use this resolution value:
        self.state_resolution = float("1.0e-2")

    # -- motion_constraint :- this will indicate whether some axis needs to be fixed for path planning:
    self.motion_constraint = "free"
    if "motion_constraint" in args:
        self.motion_constraint = args["motion_constraint"]

    # -- make the robot collection a global object:
    self.robot_collection = sim.createCollection()
    sim.addItemToCollection(self.robot_collection, sim.handle_tree, self.robot, 0)

    if "use_lua" in args: self.use_lua = args["use_lua"]

#end


def ompl_path_planning(args):
    """
    This function requires a dictionary containing the following fields:
        1. "robot" :- the name of the robot's base in the scenario
        2. "goal" :- the object handle for a target (this should be some kind of dummy object -- refer to Python code for example)
        3. "algorithm" :- the name of the motion planning algorithm to use (by default, "RRTstar" will be used)
        4. "num_attempts" :- the number of times to run OMPL (default: 20)
        5. "max_compute" :- the maximum time (in seconds) allotted to computing a solution
        6. "max_simplify" :- the maximum time (in seconds) allotted to simplifying a solution
        7. "len_path" :- the number of states for path generation (default: leave it to OMPL)
    """

    # -- parse all the arguments sent to this function and make them global variables (i.e., "self.XXX"):
    parse_args(args)

    # -- find a valid configuration that puts the robot's gripper at the goal location:
    valid_config = findIKConfig(args)

    ################################################################################################

    print(valid_config)

    final_path = None

    if len(valid_config) > 0:
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
        simOMPL.setStateValidityCheckingResolution(self.ompl_task, self.state_resolution)

        if self.use_state_validation:
            # WARNING: state validation is slow, probably best not to use it:

            if self.motion_constraint == "free":
                if self.verbose: sim.addLog(sim.verbosity_default, "\t-- Considering free-axis orientation...")
                simOMPL.setStateValidationCallback(self.ompl_task, stateValidationCollision_python)
            elif self.motion_constraint == "fix-x":
                if self.verbose: sim.addLog(sim.verbosity_default, "\t-- Considering fixed-axis orientation...")
                # TODO: fix this:
                #simOMPL.setStateValidationCallback(self.ompl_task, stateValidation)

        simOMPL.setup(self.ompl_task)

        for _ in range(self.num_ompl_attempts):
            # -- read more about compute operation here: https://manual.coppeliarobotics.com/en/simOMPL.htm#compute
            if self.use_lua:
                data = {
                    "ompl_task": self.ompl_task,
                    "max_compute": self.max_compute,
                    "max_simplify": self.max_simplify,
                    "len_path": self.len_path,
                    "use_state_validation": self.use_state_validation,
                }
                output = sim.callScriptFunction("luaOMPLCompute", sim.handle_self, data)
                self.ompl_task, result, path = output[0], output[1], output[2]
            else:
                result, path = simOMPL.compute(
                    self.ompl_task,
                    self.max_compute,
                    self.max_simplify,
                    self.len_path,
                )

            # -- we will see if there was an exact solution found;
            #    that way we know if we might need to loop back around again to find the solution
            is_exact_solution = simOMPL.hasExactSolution(self.ompl_task)
            sim.addLog(sim.verbosity_default, f"[OMPLement] : Exact solution found? -- {is_exact_solution}")

            sim.addLog(sim.verbosity_default, f"[OMPLement] : Distance to target -- {simOMPL.getGoalDistance(self.ompl_task)}")

            # -- if no exact solution was found... then maybe we will compute again?

            if result and is_exact_solution:
                # -- We found a collision-free path!
                sim.addLog(sim.verbosity_default, f"[OMPLement] : Length of path: {int(simOMPL.getPathStateCount(self.ompl_task,path))}")

                simOMPL.printTaskInfo(self.ompl_task)

                # NOTE: the path contains a Mx1 vector, which needs to be transformed to NxJ vector, where N = M/J.
                # -- the final path will be stored as a NxJ matrix, where N = number of points in trajectory and J = number of joints.
                final_path = []

                for x in range(0, len(path), len(self.joint_handles)):
                    final_path.append(path[x:x+len(self.joint_handles)])

                # NOTE: old way of parsing through the path -- it is very slow
                #for i in range(int(simOMPL.getPathStateCount(ompl_task,path))):
                #    conf=simOMPL.getPathState(ompl_task,path,i+1)
                #    final_path.append(conf)

                assert simOMPL.getPathStateCount(self.ompl_task,path) == len(final_path), "[OMPLement] : error in path rebuild?"

                break

        simOMPL.destroyTask(self.ompl_task)

    else:
        sim.addLog(sim.verbosity_scriptwarnings, "[OMPLement] : no configuration found!")

    return final_path