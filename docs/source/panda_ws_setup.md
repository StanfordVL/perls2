# Set up workstation for Franka Panda

## Description
The Workstation hosts the perls2 environment and sends robot commands to the NUC via the redis-server. The redis-server is connected via TCP.

## Installation and setup
1. Install redis
```
sudo apt-get install redis-server
```

2. Clone and install [perls2](https://gitub.com/StanfordVL/perls2)

3. Copy the redis_passfile.txt from the nuc to your workstation.

4. Modify the perls2/cfg/redis.yaml `password` key to the filepath for your redis-passfile.

5. Follow the steps [here](panda_instructions.md) for using the Franka Panda, and run the Gravity Compensation demo.
    ```
    python perls2/demos/run_gc_demo.py --Real
    ```
