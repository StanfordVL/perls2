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

4. [Clone and install perls2](introduction.md#installing)

5. In perls2, checkout the `panda_dev` branch

    ```
    git checkout panda_dev
    ```

6. Install redis.

    `sudo apt-get install redis-server`

7. Install the [franka-panda-iprl driver.](https://github.com/StanfordVL/franka-panda-iprl/) This is used by perls2 as a redis-wrapper for libfranka. Please note that this version of the driver has been modified specifically to work with perls2.

### Secure your redis server
Because redis-server process requests very quickly, it is critical to secure your redis-server by setting a password. It's best to set a very long, randomly generated password.

1. Copy the redis.conf file found in perls2/redis_interfaces/redis.conf to the NUC's home directory. This conf file contains the redis-server password, so we don't want it online.

2. Edit the line "requirepass foobared" to "requirepass < your_password_here > ".

3. Copy the redis_passfile.txt from perls2 to a hidden folder in your home directory. Change the text to the password you entered in the redis.conf

4. Modify the perls2/cfg/redis_nuc.yaml file by changing the value of the `password` key to the filepath of your redis_passfile.txt. E.g.
    ```
    redis:
        host: 127.0.0.1
        port: 6379
        password: '/home/user/.hidden/redis_passfile.txt'
    ```

5. Modify the perls2/cfg/franka-panda.yaml by changing the value of the `password` key just as you did for perls2.

6. Run your redis-server with the conf file you've copied from perls2. If you don't use the conf file, your redis-server will not be secured.

    ```
    redis-server /path/to/.hidden/redis.conf
    ```
7. Verify that your redis-server is secured by opening [redis-cli](redis.md) in another terminal

    ```
    redis-cli -h <host_ip> -p <port>
    ```
    For example:

    ```
    redis-cli -h 127.0.0.1 -p 6379
    ```
8. Running any command in redis-cli should respond with a message 'AUTH Required':
    For example:

    ```
    > keys *
    (error) AUTH Required
    ```

9. You can enter a password for redis-cli by using AUTH followed by the password you set in the conf file.

    ```
    > AUTH your-password
    OK
    ```

### Verify perls2 NUC setup by running PandaCtrlInterface
1. Start the redis-server

    ```
    redis-server /path/to/.hidden/redis.conf
    ```

2. In the Desk app, unlock the robot. The lights at the base of the robot should be white.

3. Pull the user-stop so the lights are blue.

4. Start the franka-panda driver using the perl2 config.

    ```
    cd franka-panda-iprl/bin
    ./franka-panda-iprl ~/perls2/cfg/franka-panda.yaml
    ```
5. Open another terminal and source the perls2 environment.

6. Run the Panda Control Interface

    ```
    cd ~/perls2
    python perls2/ctrl_interfaces/panda_ctrl_interface.py
    ```

7. You should see the message:
    `Waiting for perls2.RobotInterface to connect`

8. Exit the Ctrl Interface with `Ctrl + C`

9. Exit the Driver with `Ctrl + C`