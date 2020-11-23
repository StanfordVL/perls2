"""Class to mimic franka-panda redis driver.
"""
from perls2.utils.yaml_config import YamlConfig
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
import pytest
P = PandaKeys('cfg/franka-panda.yaml')


class FakeFrankaPanda(object):
    """Class for faking panda redis driver.

    Sets fake values for redis, mocking franka-panda.
    """
    FAKE_STATE = {
        P.DRIVER_CONN_KEY : P.DRIVER_CONNECTED_VALUE, 
        P.ROBOT_STATE_Q_KEY: "0.153399527304672 -0.0228098171871691 -0.0856113330690632 -2.13645188983878 0.000673128167293589 2.11842455338582 2.61612591026992",
        P.ROBOT_MODEL_CORIOLIS_KEY : "7.77772210829083e-07 -1.5204640511763e-06 8.42576906897586e-07 -1.0812855712552e-06 -7.18757666207027e-08 -2.71478306345705e-07 1.80225862506108e-09",
        P.CONTROL_MODE_KEY: P.TORQUE_CTRL_MODE, 
        "franka_panda::sensor::dtau": "-28.4223251342773 -37.3109169006348 134.719436645508 -11.3944501876831 -1.251708984375 33.213264465332 -119.295654296875",
        "franka_panda::control::tau": "0.0001 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001", 
        "franka_panda::control::pose": "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1", 
        "franka_panda::sensor::tau" : "-0.349489659070969 -29.2690124511719 0.170498847961426 22.5394229888916 0.614682018756866 2.68961238861084 0.410876989364624",
        "franka_panda::sensor::dq" : "0.000125559134503359 0.000990025413722996 -0.000505009065291245 0.0027716119003343 0.00193538339277136 0.075633681715534 1.03529999095642",
        "franka_panda::gripper::control::width": "0.0805487",
        "franka_panda::sensor::pose" : "-0.191278464901784 -0.981519220155279 -0.00364914124110978 0 -0.98151560931327 0.191292227149249 -0.00389093920386819 0 0.00451717093778475 0.00283749084128745 -0.999985771805002 0 0.534449340443935 0.0364261218566121 0.313053490964604 1",
        "franka_panda::gripper::control::force": "0",
        "franka_panda::gripper::status": "off", 
        "franka_panda::control::status": "running",
        "franka_panda::gripper::model::max_width": "0.0805658",
        "franka_panda::model::jacobian": "-0.0364261218566121 0.534449340443935 0 0 0 1 -0.0197122840892743 -0.0030477989899048 -0.53373934800645 -0.152798616765289 0.988257346400529 0 -0.0363471325252235 0.53386071813922 0.00104151399259117 -0.0225400147376614 -0.00348500629550397 0.999739867398888 0.337024510725762 0.0219889104042853 0.460390546029072 0.0677582709121419 -0.997699861353456 -0.00195022519003353 -0.00902050122648396 0.133508259078352 0.000338086126741867 0.85416294415736 0.0590199327787439 -0.516645248079592 0.209521735234418 0.0139310161853758 0.0889872407518848 0.0674112095558713 -0.997722078208546 -0.00252655525070112 -1.0842021724855e-19 1.0842021724855e-19 0 0.00451717093778475 0.00283749084128745 -0.999985771805002",
        "franka_panda::model::mass_matrix": "1.37243179077682 0.0303098401746899 1.35593638500276 -0.0146822820059845 0.0173479147160189 -0.00126769964125797 -0.00414517617602964 0.0303098401746899 2.06766540614763 0.0241402874034015 -0.997500154723087 -0.0176547834430898 -0.123857981105795 -0.00145879120569034 1.35593638500276 0.0241402874034015 1.36904228873836 -0.0131312265850373 0.0170994836314575 -0.00107255945243665 -0.00410900449545804 -0.0146822820059845 -0.997500154723087 -0.0131312265850373 1.07031400826678 0.0243980317964721 0.166687043861745 0.00182397437507655 0.0173479147160189 -0.0176547834430898 0.0170994836314574 0.0243980317964721 0.0114169574657517 0.00222519688706938 7.43949949634922e-05 -0.00126769964125797 -0.123857981105795 -0.00107255945243665 0.166687043861745 0.00222519688706938 0.0482222181084137 0.00173273660855194 -0.00414517617602964 -0.00145879120569034 -0.00410900449545804 0.00182397437507655 7.43949949634922e-05 0.00173273660855194 0.00296417336813313",
        "franka_panda::timestamp::robot": "6.34547e+07",
        "franka_panda::gripper::sensor::width": "0.0805487",
        "franka_panda::model::gravity": "-7.02020906684364e-18 -29.5563823741208 0.0363060964400721 22.8941858072261 0.653781343181317 2.47546341060765 -2.06027061716643e-05",
        "franka_panda::gripper::control::grasp_tol": "0.02 0.02\n",
        "franka_panda::model::inertia_ee": "{\"I_com_flat\":[0.0010000000474974513,0.0024999999441206455,0.0017000000225380063,0.0,0.0,0.0],\"com\":[-0.009999999776482582,0.0,0.029999999329447746],\"mass\":0.8500000238418579}",
    }
    def __init__(self):
        self.driver_config = YamlConfig('cfg/franka-panda.yaml')
        # some renaming for the panda redis interface.
        redis_config = {"host": self.driver_config['redis']['ip'],
                        "port": self.driver_config['redis']['port'],
                        "driver_config":'cfg/franka-panda.yaml' }
        self.redisClient = PandaRedisInterface(**redis_config)


    def start(self):
        self.redisClient.set(P.DRIVER_CONN_KEY, P.DRIVER_CONNECTED_VALUE)

    def stop(self):
        self.redisClient.set(P.DRIVER_CONN_KEY, P.DRIVER_DISCONN_VALUE)

    def set_q(self, q):
        """ Set fake joint positions in format franka-panda redis uses

        Args:
            q (list): 7f joint positions.
        """
        self.redisClient.set(P.ROBOT_STATE_Q_KEY, str(q))

    def set_states(self, states):
        """Set robot state via redis, using franka-panda format for strings.
        """
        self.redisClient.mset(
            {P.ROBOT_STATE_Q_KEY: states["q"],
             P.ROBOT_STATE_DQ_KEY: states["dq"],
             P.ROBOT_STATE_EE_POSE_KEY : states["pose"],
             P.ROBOT_STATE_TAU_KEY : states["tau"]}
             )

    def set_fake_state(self): 
        """Set redis database to predefined fake state taken from real
        franka driver
        """

        self.redisClient.mset(self.FAKE_STATE)

######################### TESTING #######################################3
@pytest.fixture()
def driver():
    return FakeFrankaPanda()

def test_start(driver):
    driver.start()
    assert(driver.redisClient.get(P.DRIVER_CONN_KEY).decode() == P.DRIVER_CONNECTED_VALUE.decode())

def test_set_fake_state(driver):
    driver.set_fake_state()
    assert(driver.redisClient._client.get(P.ROBOT_STATE_Q_KEY).decode() == \
        FakeFrankaPanda.FAKE_STATE[P.ROBOT_STATE_Q_KEY])
    assert(driver.redisClient._client.get(P.ROBOT_STATE_EE_POSE_KEY).decode() == \
        FakeFrankaPanda.FAKE_STATE[P.ROBOT_STATE_EE_POSE_KEY])
