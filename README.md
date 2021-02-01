# perls2
PErception and Robotic Learning System v2

perls2 provides robot software infrastructure for projects within SVL. It serves as a unified framework that connects both simulated and real robots, sensors, renders and simulation environments for performing robotics experiments. 

The design document describes the goals, specifications and milestones for perls2 development. 

[Website and Read the docs](https://stanfordvl.github.io/perls2/)

[Design Document](https://docs.google.com/document/d/1JJA4TpnnS4lhWyXyyhaU3PcngXAHG2iap1pUcpQy9wY/edit)

New to perls2? [Start Here](https://stanfordvl.github.io/perls2/source/introduction.html)

## Installing
PERLS2 only supports python3.6 for the core libraries (python 2.7 is used for interfacing to robots that use ROS.)
PERLS2 only supports ubuntu 16.04 and later.

### Installing python 3.6
```
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt-get install python3.6
```

### On Ubuntu 16.04
1. Clone and install PERLs repo
    1. Clone repo: 
        
        `git clone https://github.com/StanfordVL/perls2.git`
    2. Create a virtual environment, e.g. 
        
        `virtualenv -p python3.6 perls2env`
    3. Source your virtual environment e.g. 
        
        `source perls2env/bin/activate`

    4. Go to the perls2 directory and install requirements
        
        `cd ~/perls2`
        
        `pip install -r requirements.txt`

    5. Install perls2 
        
        `pip install -e .`

## Run an example
Check out the examples in the `examples` folder [Overview](https://github.com/StanfordVL/perls2/tree/master/examples)

Intructions for each demo may be found in the README in each folder.
### Example: Run simple reach demo
`cd perls2`

`python examples/simple_reach/run_simple_reach.py`

## Setup for real robot control
Instructions for setting up perls2 with real robot control are specific to machine and robot type. 

[Setup instructions for Franka Panda](https://stanfordvl.github.io/perls2/source/panda_setup.html)
[Setup instructions for Rethink Sawyer](https://stanfordvl.github.io/perls2/source/sawyer_instructions.html)

