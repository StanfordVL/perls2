# perls2
PErception and Robotic Learning System v2

PERLS provides robot software infrastructure for projects within SVL. PERLS allows for a unified framework that enables robotics research on both simualted and real robots. PERLS2 is a redesign with the goal of offering only essential functionality for robotics research. Please refer to the PERLS Design Document for more information. 

TODO: Design Doc Link

TODO: API Link

TODO: Examples Links

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Installing
#### For Mac OS X
PERLS2 only supports python3.7 for the core libraries (python 2.7 is used for interfacing to robots that use ROS.)
1. Clone and install PERLs repo
    1. Clone repo: 
        
        `git clone https://github.com/StanfordVL/perls2.git`
    2. Create a virtual environment, e.g. 
        
        `virtualenv -p python3.7 perls2env`
    3. Source your virtual environment e.g. 
        
        `source perls2env/bin/activate`
#### On Ubuntu 16.04
PERLS2 only supports python 3.6 virtual environments (due to a dependency on RBDL). 

1. Clone and install PERLs repo
    1. Clone repo: 
        
        `git clone https://github.com/StanfordVL/perls2.git`
    2. Create a virtual environment, e.g. 
        
        `virtualenv -p python3.6 perls2env`
    3. Source your virtual environment e.g. 
        
        `source perls2env/bin/activate`
        
#### For all installs.
    4. Install RBDL and required dependencies by following the instructions [here](https://github.com/StanfordVL/rbdl/wiki/Perls2-RBDL-Install-Instructions)
    
    5. Go to the perls2 directory and install requirements
        
        `cd ~/perls2`
        
        `pip install -r requirements.txt`
        * You could get an error like `error: numpy 1.11.0 is installed but numpy>=1.11.1 is required by set(['opencv-python'])`. In that case you may want to force to install the right version of numpy: `pip install --upgrade --force-reinstall numpy`

    6. Install perls2 
        
        `pip install -e .`
## Run PERL demos
Check out the  demos in the examples folder. Intructions for each demo may be found in the README in each folder.
### Example: Run simple reach demo
`cd perls2`

`python examples/simple_reach/run_simple_reach.py`





