# Switching between Real and Sim Example

## Description
The Switching between Real and Sim example is meant to demonstrate how to share logic between real and simulated experiments. A reach task is performed in pybullet that uses the object interface. A reach task for a randomly generated position is performed on the real robot. Both examples use the Sawyer Robots.

There are three files worth looking at for this demo:
* Run file: run_switch_sim_real.py
* Environment file: switch_sim_real_env.py
* Config file: simple_reach.yaml

To run this demo in sim, go to the configuration file and set the `world:type` key to 'Bullet'.

    `python run_switch_sim_real.py`

To run this demo in real, change the `world:type` key from 'Bullet' to 'Real'. Then check the wiki for instructions about running the real Sawyer robot. Set up those terminals and then in another terminal run
    `python run_switch_sim_real.py`

## Run file:
The run file is almost identical to the simple_reach example. The only change is the constructing the environment. The same analytical policy is used for `get_action`.

## Environment file:
The Environment file is also similar to the simple_reach example. To wrap code for simulation/real, we use the `self.world.is_sim` flag. In simulation, our goal position is determined by the pose of the object, obtained from the `object_interface`. In reality, our goal position is randomly generated within some bounds set in the configuration file.

## Config file:
The key configuration parameter in this example is `world:type`. To change from a Bullet world to real change this from 'Bullet' to 'Real'.

The `goal_position` key is a user defined parameter that limits the range of the randomly generated goal position for the real experiment.


