# Introduction

PErception and Robotic Learning System v2

perls2 provides robot software infrastructure for projects within SVL. It serves as a unified framework that connects both simulated and real robots, sensors, renders and simulation environments for performing robotics experiments.

New to perls2? [Start Here](https://stanfordvl.github.io/perls2/quickstart)

## Why use perls2?

perls2 makes it easier to run experiments on real robots. Code for running experiments is agnostic to the simulation / reality. This means that code written for simulation can be made to run on real robots, just by changing a single config key. It is also easy to switch between the different robots offered in the lab.

Additionally, perls2 offers consistent robot control between simulated and real robots through a unified torque-based library of robot controllers. This makes it easy to explore different action spaces and forms for robot control. Current controllers include:

- Joint Impedance
- Operational Space Control
- Joint Velocity
- Joint Torque

All controllers have been validated to meet specifications and performance metrics.


## Installing
PERLS2 only supports python3.6 for the core libraries (python 2.7 is used for interfacing to robots that use ROS.)
PERLS2 only supports ubuntu 16.04 and later.
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
## Run PERL demos
Check out the examples in the `examples` folder [Overview](https://github.com/StanfordVL/perls2/tree/master/examples)

Intructions for each demo may be found in the README in each folder.
### Example: Run simple reach demo
`cd perls2`

`python examples/simple_reach/run_simple_reach.py`
