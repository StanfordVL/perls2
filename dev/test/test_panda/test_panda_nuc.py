"""Test script for PandaCtrlInterface <-> RealPandaInterface

This script is meant to be run on the nuc, may be extended to ws.

## Instructions.
1. Start local redis-server
```bash
    redis-server
```
"""
import redis
import pytest
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from perls2.ros_interfaces.redis_keys import *
from perls2.ros_interfaces.panda_ctrl_interface import PandaCtrlInterface
from perls2.robots.real_panda_interface import RealPandaInterface

from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.utils.yaml_config import YamlConfig
P = PandaKeys('cfg/franka-panda.yaml')

from dev.test.test_panda.fake_franka_panda import FakeFrankaPanda

def test_panda_ctrl_setup():
    # Set up fake franka panda redis driver
    redis_driver = FakeFrankaPanda()
    redis_driver.start()

    panda_ctrl = PandaCtrlInterface(config='cfg/panda_ctrl_config.yaml',
                                    controlType='EEImpedance')





if __name__ == '__main__':
    test_fake_redis_driver()
    test_panda_ctrl_setup()
    test_real_panda_setup()