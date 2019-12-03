# World

A World encompasses the simulation/reality of the experimental setup. Worlds contain specifications for the physics of simulation (gravity, timesteps, collisions.) Worlds contain arenas and interfaces to robots, objects and sensors. Worlds do **not** typically contain information relevant to the task(i.e. reward functions, observations, actions, terminations.) Environments take the state of the World to produce an observation, and apply actions to the world at each step. 
    
    
## Hierarchy: 

## Attributes:
-**config**: dict
            Config files contain parameters to create an arena, robot interface,
            sensor interface and object interface. They also contain specs for
            learning, simulation and experiment setup.
 
-**arena**: Arena
            Manages the sim by loading models (in both sim/real envs) and for
            simulations, randomizing objects and sensors parameters

-**robot_interface**: RobotInterface
            Communicates with robots and executes robot commands. Robot Interfaces are also typically constructed by the robot_factory.

-**sensor_interface**: SensorInterface
            Retrieves sensor info and executes changes to params    
 
-**object_interface**: ObjectInterface
            Only for Sim environments-- retrieves object info and excecutes
            changes to params
