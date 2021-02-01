# perls2 redis_interfaces

## Description:
The perls2.redis_interfaces module implements a wrapper around redis-py to
facilitiate communication with a redis-server.

Using the perls2 redis_interface ensures consistency between robot_interfaces
and ctrl_interfaces, and automates parsing of values obtained from redis.

The following details about the perls2 redis_interfaces are listed below and primarily of interest for developers setting up a new robot or camera.

## Redis
"Redis is an open source (BSD licensed), in-memory data structure store, used as a database, cache, and message broker. Redis provides data structures such as strings, hashes, lists, sets, sorted sets with range queries, bitmaps, hyperloglogs, geospatial indexes, and streams."

For more information see: https://redis.io/


## Redis and perls2
perls2 uses redis primarily for IPC (interprocess communication) with real devices (robots, cameras, sensors). perls2 connects many different devices that often run simultaneously and need to share data.

### Workstation - NUC communication

For real robot control, and especially at the frequency required for Torque control (~500Hz - 1kHz), it is necessary to run the robot controller driver on a separate computer with an RT kernel to keep control loop timing consistent.
Typical robot experiments with vision or learning involve some form of image processing or other computation and time-sensitive resources that prohibit them from being run on the same machine as the robot's control loop.


perls2 uses the redis interface to communicate between the workstation that runs the policy and the NUC which runs the robot controller.

