Simple Reach Example
====================

# Simple Reach Example

## Description
The simple reach example is meant to demonstrate the basic components of a PERLS2 project. It also shows how perls2 streamlines getting a simulation with a robot up and running. This example only uses PyBullet.

There are three files worth examining for this demo:
* Config file: simple_reach.yaml
* Environment file: simple_reach_env.py
* Run file: run_simple_reach.py

## Run file:
The run file shows how an environment can be combined with a policy to run an experiment. The envrionment is defined using SimpleReachEnv and outputs observations, reward, termination and info. This is similar to the OpenAI.gym pattern. The observations at each step are given to a dummy policy described in `get_action(observation)` that uses the observation at the step to produce an action.

The Bullet Environment is initialized to use the visualizer by default, but this can be changed in the construction of the SimpleReachEnv.

## Environment file:
The environment file contains code specific to managing the environment and describing the task. The main functions worth taking a look at are:
* `reset` : reset the environment after completion of the episode, randomization of object placements and sensor parameters happens here.
* `step`: execute an action and step the environment forwards.
* `rewardFunction`: calculate the reward to be given to the agent based on the state of the environment.

Some helper functions also exist to help organize your code:
* `_exec_action` : how the environment should interpret and execute the action. In this example, the action is a delta between the current end effector position of the robot and some desired position. This is executed by setting the robots new position to its current position + this delta.
* `get_observation`: the observation the environment should provide. Our dummy policy only uses the delta end effector position from the goal, but you can also get the current end_effector pose, and a rendered image at the step.

* `_check_termination`: returns whether or not the episode has completed, either by execeeding the maximum number of steps or if the goal has been acheived.

## Config file:
The configuration file contains the parameters that describes the robot, the sensors used and the objects it will interact with. The configuration file is broken up into a few components

### robot
For right now, it is not advised to edit with the robot configuration as these describe the setup of the robot. Please see more advanced demos

### sensors
These config parameters describe sensor behavior.  In this example, we only have a camera. You can try changing the properties in the intrinsics/extrinsics to see how they change the rendered images in bullet. These will show up on the left side of the rendered image.

TODO: add pictures highlighting rendered image

The configuration parameters also prescribe the randomization behavior of the camera. Under the `random` key there are a few keys of interest
* `randomize`: True/False to randomize all parameters upon reset.
The intrinsic and extrinsic parameters under this key all contain `lower` and `upper` keys that allow you to define the min and max range for randomization. For example

`extrinsics:
  eye_position:
    lower:
      [0.3, 0., 1.5]
    upper:
      [0.5, 0., 2.0]`

  means that for the eye position extrinsic property (where the camera is located) the x position is bounded from [0.3, 0.5], y position is [0.0, 0.0] and z position is [1.5, 2.0]

 You can modify these ranges to see how it changes the rendered image on reset.

### objects
Object parameters define what type of object, where it is placed and if it can move. These can be modified in a similar manner to sensor parameters. Of particular interest is the `is_static` key, which determines if the object can move. The randomization parameters can be modified similarly to the sensor parameters.

Try modifying the is_static key and changing the type of object to see how the simulation changes.
