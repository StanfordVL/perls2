# Safenet ReadME

## Description
This example demonstrates how to set up and use Safenet for robot experiments. Safenet provides a 
workspace boundary for a robot. All goals sent to the controller are clipped to this boundary. 
A python script for recording workspace boundaries on the real robot is also [provided.](#Generate_safenet_config_for_your_workspace)


## Safenet usage: 
There are two ways to set up a safenet boundary. Either using a config file, or via function call. 
The boundary positions consist of the min / max x, y and z positions for the end-effector. Any goals
commanded to the robot that set the goal for the controller outside these bounds, will be clipped to 
the boundary.

### Set up Safenet from Config
The config file [safenet.yaml](safenet.yaml) shows the relevant parameters for defining a boundary. 


```yaml
safenet: 
  use_safenet: False
  lower: [0.35, 0.06, 0.15]
  upper: [0.55, 0.26, 0.31]
```

In order for the Env to automatically set up the safenet from config, `use_safenet` key must have the value `True`

## Set up safenet via function call: 
To set up safenet after a Robot Interface has been created: 

```python
lower_limit = [0.35, 0.06, 0.15]
upper_limit = [0.55, 0.26, 0.31]

env.robot_interface.set_safenet_boundaries(lower_limit, upper_limit)
```

**Note**: for real robots, changing the safenet boundary results in a CHANGE_CONTROLLER command, which can only be performed after robot has reset. Controller changes cannot be made while the robot is executing commands.

You can get the current safenet boundaries as a tuple. 
```python
(upper, lower) = env.robot_interface.get_safenet_limits()
```

## Run SafeNet Example. 
It's best to start the safenet example in PyBullet. Make sure the `world` key in the [safenet_env.yaml](safenet_env.yaml) config is set to 'Bullet'. 

Because workspace setups and robot geometries differ, this example sets the workspace boundary from the initial end-effector pose out of reset. 

The SafeNet box is visualized in Red. A blue path indicates the commanded path for the robot, which exceeds the bounds of the workspace. The robot remains in the safenet bounds. 

This example should also work for real robots to. Just follow the setup to start your real robot, and run the example with the `world: 'Real'` in the safenet_env.yaml config.

## Generate safenet config for your workspace
You can generate a config for a safenet by moving the robot end-effector to the workspace boundaries and recording end-effector positions. 
After setting up perls2 for your real robot, use the `get_safenet_boundary.py` script to record workspace boundaries. You can supply a path
for the output yaml file.
	
```bash
	cd ~/perls2
	source ~/p2env/bin/activate
	python examples/safenet/get_safenet_boundary output/my_safenet.yaml
```

To include these boundaries in your project, just copy the contents or add the following line to your main perls2 config file. 
```yaml
	!include /relative/path/to/my_safenet.yaml
```

