# Set up NUC (Control PC) for the Franka Panda

## Description
The NUC or Control PC runs three processes to control the Franka Panda arm.

1. franka-panda-iprl driver : A wrapper for libfranka that gets torque commands from redis and updates the redis-server with robot states.

2. perls2.PandaCtrlInterface: A process that grabs robot commands from the perls2 environment hosted on the workstation, and robot states from the franka-panda-iprl driver. This process implements the controller and calculates the torques necessary to acheive the desired robot states, setting the torque command to redis.

3. redis-server : A redis server is used to communicate between the franka-panda-iprl redis driver (a wrapper for libfranka) and the perls2.PandaCtrlInterface


## Important notes:
To ensure consistent control loop timing, it is important that no other processes be running on the NUC. This includes internet browsers or text editors. After initial set up, the NUC should be accessed only by ssh from the workstation.


## Setting up the NUC
1. [Set up the Ubuntu 16.04 with RT-PREEMPT Kernel](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)

2. [Set up the Franka Panda Arm and Network.](https://frankaemika.github.io/docs/getting_started.html#control-network-configuration) There is no need to install libfranka just yet, the driver will do this later. It's best to use a high bandwith network switch to connect the workstation, the FCI and the NUC to a local network.

3. [Disable cpu power scaling](https://frankaemika.github.io/docs/troubleshooting.html#disabling-cpu-frequency-scaling) using cpufrequtils by following the [instructions here] The default setting slow down the CPU frequency for power conservation. We want our control loop to run as fast as possible to maintain a high frequency for torque control.


4. Clone perls2 and checkout the `panda_dev` branch
    ```bash
    git clone https://github.com/StanfordVL/perls2.git
    cd ~/perls2
    git checkout panda_dev
    ```
5. [Install perls2](introduction.md#installing)

6. Install redis.

    `sudo apt-get install redis-server`

7. Install the [franka-panda-iprl driver.](https://github.com/StanfordVL/franka-panda-iprl/) This is used by perls2 as a redis-wrapper for libfranka. Please note that this version of the driver has been modified specifically to work with perls2.

### Set up local directory for the Control PC
Setting up the control PC requires certain config specific to the machine. These include the redis-server password and the ip of the Franka Arm. To ensure these configs do not get overwritten when updating perls2, it's best to place them in a separate directory. This also makes it easier to use bash scripts to start the robot. 

1. Copy the `perls2_local_control_pc` directory to your home directory or somewhere outside the main perls2 repo. 
    ```bash
    cd ~/perls2;
    cp -r local/perls2_local_control_pc ~/
    ```
2. Move to the newly copied local directory;
    ```bash
    cd ~/perls2_local_control_pc
    ```
This folder contains the following files: 

* `redis.conf`              : config file used by redis-server.
* `redis_passfile.txt`      : (deprecated) password to authenticate clients to redis-server
* `redis_nuc.yaml`          : yaml config used by perls2 to connect to redis-server
* `franka-panda.yaml`       : config used by franka-panda-iprl driver.
* `panda_ctrl_config.yaml`  : default config used by perls2 panda control interface.
* `local_dir.sh`            : bash script to help find local perls2 and driver install 


#### Bind your redis server
Binding the redis-server ensures that it only allows connections from certain ip addresses. This allows us to secure the redis-server for a local network.

1. Determine the control PC's local ip address. If you followed the instructions on the Franka website, it should be 172.16.0.1. Otherwise use ifconfig. 
Note the inet addr  for the **local** network, likely 'eno1'. For example:
```bash
$ ifconfig
eno1      Link encap:Ethernet  HWaddr 94:c6:91:aa:91:80  
          inet addr:172.16.0.1  Bcast:172.16.0.255  Mask:255.255.255.0
```


2. Using your favorite text editor, open the `redis.conf` file found in the local directory you just copied.
    ```bash
    cd ~/perls2_local_control_pc
    nano redis.conf
    ```

3. Edit the line 68 `bind 127.0.0.1 172.16.0.1` and replace the "172.16.0.1" with your control pc ip address if needed. Make sure you have a space between the two addresses, and don't delete the "127.0.0.1".
```
# Examples:
# bind 127.0.0.1 [Control PC Local IP]
bind 127.0.0.1 172.16.0.1
```

#### Modify config for driver: 
1. Open the `franka-panda.yaml` config file in a text-editor. 
    ```bash
    cd ~/perls2_local_control_pc
    nano franka-panda.yaml
    ```

2. Edit the `robot_ip` key to the ip address of the Franka Master Controller
    ```yaml
    ####################
    # Robot parameters #
    ####################

    robot:
      ip: "172.16.0.2"
    ```

#### Set up for bash scripting
Configuring the `local_dirs.sh` file makes it convenient to start the robot by using bash scripts. 

1. In the perls2_local_control_pc directory, open the local_dirs.sh file. 
    ```bash
    cd ~/perls2_local_control_pc
    nano local_dirs.sh
    ```

2. Edit following variables in the script with directories and file locations specific to your pc. If you set up everything in your home directory, you shouldn't have to modify anything.
    ```bash
    #!/bin/bash
    # Command to source virtualenv environment with perls2 installed
    export SOURCE_ENV_CMD="source ~/p2env/bin/activate"
    # Directory for perls2 repo
    export PERLS2_DIR="$HOME/perls2"
    # Directory for franka-panda-iprl repo
    export DRIVER_DIR="$HOME/franka-panda-iprl"
    ```

3. Make the file executable with: 
    ```bash
    chmod a+x local_dirs.sh
    ```

### Verify perls2 NUC setup using bash scripts:
1. In the Desk app, unlock the robot. The lights at the base of the robot should be white.

2. Pull the user-stop so the lights are blue. 

3. Move to the perls2 directory. 
    ```bash
    cd ~/perls2
    ```
4. Run the `start_panda_control.sh` script passing the path to the `perls2_local_control_pc` folder you copied as an argument: 
    ```bash
    ./scripts/start_panda_control.sh ~/perls2_local_control_pc
    ```
5. You should see the message:
    `Waiting for perls2.RobotInterface to connect`

6. Exit the Ctrl Interface with `Ctrl+C`
