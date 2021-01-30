# Redis Help

## Overview
perls2 uses redis for interprocess communication and retreiving data from real robots and sensors with different software requirements, as well as communication between machines for real robot control. This page lists a number of tips for working with redis.

## Redis
"Redis is an open source (BSD licensed), in-memory data structure store, used as a database, cache, and message broker. Redis provides data structures such as strings, hashes, lists, sets, sorted sets with range queries, bitmaps, hyperloglogs, geospatial indexes, and streams."

For more information see: https://redis.io/

For real robot control, processes communicate by writing or reading the value of specific redis keys. These keys contain information about control parameters, robot commands, and robot state.

## Install redis
```
sudo apt-get install redis-server
```

## Start redis-server
```
redis-server /path/to/redis.conf
```

You can run an unsecured server using `redis-server`, but be aware that this is unsecured and will not work with perls2.

## Securing redis
An unsecured redis-server can lead to a number of [vulnerabilities](https://redis.io/topics/security), which are especially important when performing real robot control. To secure the server, we modify the redis.conf found in the perls2/redis_interfaces directory.

1. Copy the perls2/redis_interfaces/redis.conf to another directory. This conf file contains the password, so it's not a good idea to have it in a github repo or anywhere online.

    ```
    cp ~/perls2/redis_interfaces/redis.conf ~/home/user/.hidden/
    ```

2. Change the following line in the conf file you just copied over

    ```
    requirepass foobared
    ```
    to

    ```
    requirepass your-super-long-password
    ```

3. In the hidden directory, make  text file containing the password you've just added to the conf file. perls2 will use this to connect to redis.

4. Follow the instructions specific to the machine and robot you're working with for linking perls2 to your redis passfile.

[Setting up NUC for Franka Panda](panda_nuc_setup.md)
[Setting up Workstation for Franka Panda](panda_ws_setup.md)

5. To run a secured redis-server, add the path to your new conf file when you start it.

    ```
    redis-server /path/to/redis.conf
    ```

## redis-cli
Redis cli is an command line interface that makes it convenient to interact with a running redis-server. redis-cli is installed when you install redis.

To connect to a redis-server with redis-cli

    ```
    redis-cli -h <host ip> -p <port number>
    ```
The host-ip for real robot control will usually be the NUC's ip address, and port number is found in your config file.

### redis-cli commands
For more information about redis-cli and commands see [this link](https://redis.io/topics/rediscli). The most relevant ones for perls2 are listed below:

* AUTH password : If you've secured your server, you won't be able to use redis-cli without authenticating
    ```
    > AUTH your-redis-server-password
    OK
    ```

* GET key: get the value of a key.

    Example:
    ```
    > get franka_panda::control::tau
    ```

* SET key value : Set the value of a key
    ```
    > set franka_panda::control::mode idle
    OK
    ```

* FLUSHALL : Flush all keys and values in the database. Use before starting the driver.
    ```
    > FLUSHALL
    OK
    ```

* KEYS prefix : List all keys starting with prefix string

    Example: list all keys.
    ```
    > KEYS *
    ```
* SHUTDOWN : shut down current redis-server.

## Troubleshooting:

### Kill a persistent redis-server
Sometimes redis-servers will continue to run in the background, and you get errors saying that the port is already in use. You might notice a redis-server refuses to use the .conf file and remains unsecured.

This command can be used to kill a persistent server

    **Stop redis-server**:
    ```
    /etc/init.d/redis-server stop
    ```