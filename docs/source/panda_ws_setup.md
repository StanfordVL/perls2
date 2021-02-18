# Set up workstation for Franka Panda

## Description
The Workstation hosts the perls2 environment and sends robot commands to the NUC via the redis-server. The redis-server is connected via TCP.

## Setting up SSH Key to Control PC
(Instructions copied from [HARPLAB gastronomy](https://github.com/HARPLab/gastronomy/tree/master/manipulation))
1. Generate an ssh key by executing the following commands or reading the [instructions here](https://help.github.com/en/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent):
   ```bash
   ssh-keygen -t rsa -b 4096 -C "workstation name"
   [Press enter]
   [Press enter]
   [Press enter]
   eval "$(ssh-agent -s)"
   ssh-add ~/.ssh/id_rsa
   ```
2. Upload your public ssh key to the control pc.
   1. In a new terminal, ssh to the control PC.
      ```bash
      ssh [control-pc-username]@[control-pc-ip]
      Input password to control-pc.
      ```
   2. Use your favorite text editor to open the authorized_keys file.
      ```bash
      vim ~/.ssh/authorized_keys
      ```
   3. In a separate terminal on your Workstation PC, use your favorite text editor to open your id_rsa.pub file.
      ```bash
      vim ~/.ssh/id_rsa.pub
      ```
   4. Copy the contents from your id_rsa.pub file to a new line on the authorized_keys file on the Control PC. Then save. 
   5. Open a new terminal and try sshing to the control PC and it should no longer require a password. 

## Installation and setup
1. Install redis
    ```
    sudo apt-get install redis-server
    ```

2. Clone perls2 and checkout the `panda_dev` branch
    ```
    git clone https://github.com/StanfordVL/perls2.git
    cd ~/perls2
    git checkout panda_dev
    ```
3. [Install perls2](introduction.md#installing)

4. Copy the redis_passfile.txt from the `perls2_local_control_pc` directory on your Control PC to your workstation (preferably outside the perls2 directory.)

5. If necessary, modify the perls2/cfg/redis.yaml with the ip address of the control pc on the local network: 
  ```
  # Redis-server hosted by NUC for Robot Control
  redis:
    host: [Local ip address of the NUC] 
  ```  
  
For example:
  ```yaml
  # Redis-server hosted by NUC for Robot Control
  redis:
    host: "172.16.0.1"
    port: 6379
    password: null
  ```  

## Test out a demo
1. Follow the steps [here](panda_instructions.md) for using the Franka Panda, and run the Gravity Compensation demo.
    ```
    python perls2/demos/run_gc_demo.py --world=Real
    ```
