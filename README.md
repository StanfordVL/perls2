# perls2
PErception and Robotic Learning System v2

PERLS provides robot software infrastructure for projects within SVL. PERLS allows for a unified framework that enables robotics research on both simualted and real robots. PERLS2 is a redesign with the goal of offering only essential functionality for robotics research. Please refer to the PERLS Design Document for more information. 

TODO: Design Doc Link

TODO: API Link

TODO: Examples Links

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Installing
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
        * You could get an error like `error: numpy 1.11.0 is installed but numpy>=1.11.1 is required by set(['opencv-python'])`. In that case you may want to force to install the right version of numpy: `pip install --upgrade --force-reinstall numpy`
    5. Install perls2 
        
        `pip install -e .`
## Run PERL demos
Check out the  demos in the examples folder. Intructions for each demo may be found in the README in each folder.
### Example: Run simple reach demo
`cd perls2/examples/simple_reach`

`python run_simple_reach.py`


**TODO**
## Running the tests

Individual tests for components such as interfaces should be tested after any changes are made. 
For example, to run the tester script for the BulletSawyerInterface: 

Source the environment
`source perls2env/bin/activate`

Move to the tester script directory
`cd perls2/tester/bullet_tester`

Run the tester
`python bullet_sawyer_tester`

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc

