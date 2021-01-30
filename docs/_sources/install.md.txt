# Install
PERLS2 only supports python3.6+ for the core libraries (python 2.7 is used for interfacing to robots that use ROS.)
PERLS2 only supports ubuntu 16.04 and later. Performance on MacOS not guaranteed.

### On Ubuntu 16.04
1. Clone and install PERLs repo
    1. Clone repo:

        `git clone https://github.com/StanfordVL/perls2.git`
    2. Create a virtual environment, e.g.

        `virtualenv -p python3.6 perls2env`

    3. Source your virtual environment e.g.

        `source ~/perls2env/bin/activate`

    4. Go to the perls2 directory and install requirements

        `cd ~/perls2`

        `pip install -r requirements.txt`
        * You could get an error like `error: numpy 1.11.0 is installed but numpy>=1.11.1 is required by set(['opencv-python'])`. In that case you may want to force to install the right version of numpy: `pip install --upgrade --force-reinstall numpy`

    5. Install perls2

        `pip install -e .`

2. Run the simple reach example in sim
    `python examples/simple_reach/run_simple_reach.py`

## Installation for Real Robot Control
Getting perls2 to work with real robots requires a few extra steps to set up the control pc (NUC) and workstations. See the instructions below:

[Set up instructions for Franka Panda](source/panda_setup.md)

[Set up instructions for Rethink Sawyer](source/sawyer_instructions.md)