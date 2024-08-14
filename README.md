# OMPLement
Simplifying OMPL for CoppeliaSim (_because sifting through the tutorials and forums is hard enough_).

## Overview

This is a simple script that you can add to your [CoppeliaSim](https://www.coppeliarobotics.com/) project to perform path planning. This is especially compatible with function calls through the [remote API](https://manual.coppeliarobotics.com/en/remoteApiOverview.htm) provided by CoppeliaSim.

## How to Use?

1. You will need to create a dummy object with a child script (feel free to call it what you want). In this child script, you should add everything found in the file ```OMPLement.py```.
   - Alternatively, in this repository, I provide a blank scene with a dummy object called ```OMPLement```. You can simply copy this object and paste it into your scene.

2. From your code, you will need to call the path planning function from the script using [```sim.callScriptFunction```](https://manual.coppeliarobotics.com/en/regularApi/simCallScriptFunction.htm), which requires:
   - the name of the function you wish to call from the child script (i.e., ```path_planning```)
   - a script handle (from [```sim.getScript```](https://manual.coppeliarobotics.com/en/regularApi/simGetScript.htm))
   - any number of arguments needed for the function.

3. The ```path_planning``` function requires a single (Python) dictionary (or its Pythonic equivalent in whatever language you are using) as an argument. This dictionary must have the following entries:
    - ```robot``` - the name of the base of the robot in the CoppeliaSim scene
	- ```goal``` - the object handle of the target for the robot (perhaps for grasping)
	- ```num_attempts``` - the maximum number of iterations given for the solver to find a path plan. Each iteration by default tries to find a solution within 5 seconds. You should tune the arguments for the [```sim.compute```](https://manual.coppeliarobotics.com/en/simOMPL.htm#compute) function (line 162) as you need (e.g., increasing computation time).
    - ```algorithm``` - The name of the [motion planning algorithm/solver](https://ompl.kavrakilab.org/planners.html) to use (by default, ```RRTstar``` will be used); CoppeliaSim has [several OMPL algorithms](https://manual.coppeliarobotics.com/en/simOMPL.htm#enum:Algorithm) available.


4. The function will return a list of configurations (referred to as ```path``` in this documentation).
   - If successful, ```path``` will be ```N```x```J``` matrix (or 2D list), where ```N``` is the number of points in the path and ```J``` is the total number of joints. You would then need to iterate through this list while setting the configuration of the robot using [```sim.setJointPosition```](https://manual.coppeliarobotics.com/en/regularApi/simSetJointPosition.htm) for each joint.
   - If no solution was found, ```path``` will be empty.

---

Your code in Python for instance may look like the following:
```
ompl_script = sim.getScript(sim.scripttype_childscript , sim.getObject('/OMPLement'))
path = sim.callScriptFunction(
	'path_planning',
	ompl_script,
	{
		"robot": <some_robot_name>,
		"goal": <target_obj>,
		"algorithm": <algorithm>,
		"num_attempts": <num>,
	},
)
```

## Further Details on OMPL in CoppeliaSim

To deeply understand what's happening in the code, I would recommend going through the tutorials (_yes, I know..._) that are available [online](https://manual.coppeliarobotics.com/en/pathAndMotionPlanningModules.htm). There may be specific things you require for your own planning problem, so you should definitely consult the [forums](https://forum.coppeliarobotics.com/index.php).


### Questions? Comments? Feedback?

Feel free to reach out to me via [email](mailto:dpaulius@cs.brown.edu)!