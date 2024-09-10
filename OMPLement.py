'''luaExec
function transmitToLua(data)
    ompl_task = data[1]
    robotCol = data[2]
    minDist = data[3]
    maxDist = data[4]
end

function luaStateValidation_distance(state)
    local savedState = simOMPL.readState(ompl_task)
    simOMPL.writeState(ompl_task, state)
    res, d = sim.checkDistance(robotCol, sim.handle_all, maxDist)
    is_valid = (res == 1) and (d[7] > minDist)
    simOMPL.writeState(ompl_task, savedState)
    return is_valid
end

function luaStateValidation_collision(state)
    local savedState = simOMPL.readState(ompl_task)
    simOMPL.writeState(ompl_task, state)
    local result, pairHandles = sim.checkCollision(robotCol, sim.handle_all)
    if result > 0 then
        print('Robot is colliding. Colliding pair is ' .. getAsString(pairHandles))
    end
    simOMPL.writeState(ompl_task, savedState)
    return (result > 0)
end

function luaOMPLCompute(data)
    ompl_task = data[1]
    max_compute = data[2]
    max_simplify = data[3]
    len_path = data[4]
    -- run the OMPL function from lua:
    result, path = simOMPL.compute(ompl_task,max_compute,max_simplify,len_path)
    return {ompl_task, result, path}
end
'''

def sysCall_init():
    sim = require('sim')
    simIK = require('simIK')
    simOMPL = require('simOMPL')
    math = require('math')

    self.use_lua = True
    self.verbose = True
    self.use_state_validation = True

    sim.addLog(sim.getInt32Param(sim.intparam_verbosity), "[OMPLement] : Loading OMPL motion planning script...")


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
    sim.addLog(sim.getInt32Param(sim.intparam_verbosity), "[OMPLement] : Unloading OMPL motion planning script...")


def getConfig():
    config = [-1] * len(self.joint_handles)
    for J in range(len(self.joint_handles)):
        config[J] = sim.getJointPosition(self.joint_handles[J])
    return config


def setConfig(config):
    for J in range(len(self.joint_handles)):
        sim.setJointPosition(self.joint_handles[J], config[J])


def stateValidationCallback(state):
    savedState = simOMPL.readState(self.ompl_task)
    simOMPL.writeState(self.ompl_task, state)
    res, d, *_ = sim.checkDistance(self.robotCollection, sim.handle_all, self.maxDistance)
    is_valid = (res == 1) and (d[6] > self.minDistance)
    simOMPL.writeState(self.ompl_task, savedState)
    return is_valid


def configurationValidationCallback(config):
    # -- check if a configuration is valid, i.e. doesn't collide
    # -- save current config:
    tmp = getConfig()

    # -- apply new config:
    setConfig(config)

    # -- does new config collide?
    objs_in_collision = []
    is_collision, handles = sim.checkCollision(self.robotCollection,sim.handle_all)
    #if is_collision == 1 and handles[1] not in objs_in_collision:
    #    objs_in_collision.append(sim.getObjectAlias(handles[1]))

    # -- restore original config:
    setConfig(tmp)

    if self.verbose and bool(objs_in_collision):
        sim.addLog(sim.getInt32Param(sim.intparam_verbosity), "collision found with:", objs_in_collision)

    return not bool(is_collision)


def find_ik_config(args):
        # -- first check if the robot name and goal handle have been provided to the function:
    assert "robot" in args, "[OMPLement] : Robot name not defined!"
    robot_name = args["robot"]

    self.robot = sim.getObject(f'/{robot_name}')

    # NOTE: we will create a dummy object representing the target for planning!
    # -- extract the goal object given as input to this function:
    assert "goal" in args, "[OMPLement] : Goal not defined!"
    self.goal = args["goal"]

    pose = sim.getObjectPose(self.goal, self.robot)

    # -- arm_prefix :- you can define the name format for joints (in the case where maybe there is a particular
    #   set of joints for which you want to do motion planning -- e.g., Spot robot has arm joints separate to legs)
    if "arm_prefix" in args:
        joint_prefix = f"/{args['arm_prefix']}"
    else:
        joint_prefix = f"/{robot_name}/joint"
    self.tip = sim.getObject(f'/{robot_name}/tip')

    # -- num_ompl_attempts :- we have this functionality because simOMPL.compute() can reuse previously computed data
    #   Source: https://manual.coppeliarobotics.com/en/pathAndMotionPlanningModules.htm
    num_ompl_attempts = 5
    # -- check if the number of attempts for OMPL to solve a problem has been defined:
    if "num_attempts" in args: num_ompl_attempts = args["num_attempts"]

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

    sim.addLog(sim.getInt32Param(sim.intparam_verbosity), f'[OMPLement] : Number of joints for robot "{robot_name}" - {len(self.joint_handles)}')

    # -- Prepare robot collection:
    self.robotCollection = sim.createCollection()
    sim.addItemToCollection(self.robotCollection, sim.handle_tree, self.robot, 0)

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

    #path=simIK.generatePath(ikEnv,ikGroup,ikJointHandles,ikTip,500)
    simIK.setObjectPose(ikEnv,ikGoal,ikBase,pose)

    for _ in range(num_ompl_attempts):
        # -- check here for more info on how a valid configuration is found via IK: https://manual.coppeliarobotics.com/en/simIK.htm#simIK.findConfigs
        configs = simIK.findConfigs(
            ikEnv,ikGroup,ikJointHandles,
            {
                'maxDist': 0.05,
                'maxTime': 10,
                'findMultiple': False, # -- change to True to find multiple solutions
                'pMetric': [0.05,0.05,0.05,0.01],
                'cb': configurationValidationCallback
            })

        if len(configs) > 0:
            # -- found a robot config that matches the desired pose!
            return configs[0]

    return None


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

    # -- first check if the robot name and goal handle have been provided to the function:
    assert "robot" in args, "[OMPLement] : Robot name not defined!"
    robot_name = args["robot"]

    # NOTE: we will create a dummy object representing the target for planning!
    # -- extract the goal object given as input to this function:
    assert "goal" in args, "[OMPLement] : Goal not defined!"

    algorithm = simOMPL.Algorithm.RRTConnect
    if "algorithm" in args: algorithm = args["algorithm"]

    max_compute = 5
    if "max_compute" in args: max_compute = args["max_compute"]

    max_simplify = -1
    if "max_simplify" in args: max_simplify = args["max_simplify"]

    # -- len_path :- number of states for the path (default: 0 -- we leave it to OMPL)
    len_path = 0
    if "len_path" in args: len_path = args["len_path"]

    # -- num_ompl_attempts :- we have this functionality because simOMPL.compute() can reuse previously computed data
    #   Source: https://manual.coppeliarobotics.com/en/pathAndMotionPlanningModules.htm
    num_ompl_attempts = 5
    # -- check if the number of attempts for OMPL to solve a problem has been defined:
    if "num_attempts" in args: num_ompl_attempts = args["num_attempts"]

    valid_config = find_ik_config(args)

    assert self.robot != -1, "[OMPLement] : Robot base not defined!"
    assert self.tip != -1, "[OMPLement] : End-effector tip not defined!"

    ################################################################################################

    final_path = None

    if bool(valid_config):
        # -- found a robot config that matches the desired pose!
        sim.addLog(sim.getInt32Param(sim.intparam_verbosity), "[OMPLement] : valid configuration found!")

        # -- min and max distances (only used for state validation):
        self.minDistance, self.maxDistance = float('1.0e-10'), 0.5

        self.joint_weights = [1.0] * len(self.joint_projections)

        # -- Now find a collision-free path (via path planning) that brings us from current config to the found config:
        self.ompl_task = simOMPL.createTask('ompl_task')
        simOMPL.setAlgorithm(self.ompl_task, algorithm)
        simOMPL.setStateSpaceForJoints(self.ompl_task, self.joint_handles, self.joint_projections, self.joint_weights)
        simOMPL.setCollisionPairs(self.ompl_task,[self.robotCollection, sim.handle_all])
        simOMPL.setStartState(self.ompl_task,getConfig())
        simOMPL.setGoalState(self.ompl_task,valid_config)
        simOMPL.setStateValidityCheckingResolution(self.ompl_task, float('1.5e-3'))

        if self.use_state_validation:
            # WARNING: state validation is very slow, probably best not to use it:
            if self.use_lua:
                data = [self.ompl_task, self.robotCollection, self.minDistance, self.maxDistance]
                sim.callScriptFunction('transmitToLua', sim.handle_self, data)
                #simOMPL.setStateValidationCallback(self.ompl_task, 'luaStateValidation_collision')
                #simOMPL.setStateValidationCallback(self.ompl_task, 'luaStateValidation_distance')
            else:
                simOMPL.setStateValidationCallback(self.ompl_task, stateValidationCallback)

        simOMPL.setup(self.ompl_task)

        for _ in range(num_ompl_attempts):
            # -- read more about compute operation here: https://manual.coppeliarobotics.com/en/simOMPL.htm#compute
            if self.use_lua:
                data = [self.ompl_task, max_compute, max_simplify, len_path]
                output = sim.callScriptFunction('luaOMPLCompute', sim.handle_self, data)
                self.ompl_task, result, path = output[0], output[1], output[2]
            else:
                result, path = simOMPL.compute(self.ompl_task,max_compute,max_simplify,len_path)

            # -- we will see if there was an exact solution found;
            #    that way we know if we might need to loop back around again to find the solution
            is_exact_solution = simOMPL.hasExactSolution(self.ompl_task)
            sim.addLog(sim.getInt32Param(sim.intparam_verbosity), f"[OMPLement] : Exact solution? - {is_exact_solution}")

            sim.addLog(sim.getInt32Param(sim.intparam_verbosity), f"[OMPLement] : Distance - {simOMPL.getGoalDistance(self.ompl_task)}")

            # -- if no exact solution was found... then maybe we will compute again?

            if result and is_exact_solution:
                # -- We found a collision-free path!
                sim.addLog(sim.getInt32Param(sim.intparam_verbosity), f"[OMPLement] : Length of path: {int(simOMPL.getPathStateCount(self.ompl_task,path))}")

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
        sim.addLog(sim.getInt32Param(sim.intparam_verbosity), "[OMPLement] : no configuration found!")

    return final_path