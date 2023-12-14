import functools
import rclpy
import pandas as pd
import shutil
import os
import time
import logging
import time

from pathlib import Path
from datetime import datetime

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros2topic.verb.hz import ROSTopicHz
from ros2topic.api import get_msg_class

from autoware_auto_vehicle_msgs.msg import Engage
from lifecycle_msgs.msg import TransitionEvent

class testNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.subscription_engage = self.create_subscription(
            Engage,
            '/api/autoware/get/engage',
            self.engage_callback,
            10)
        self.subscription_ss2 = self.create_subscription(
            TransitionEvent,
            '/simulation/openscenario_interpreter/transition_event',
            self.transition_callback,
            10)
        self.subscription_engage  # prevent unused variable warning
        self.subscription_ss2  # prevent unused variable warning

        self.engage = Engage()
        self.transition_event = TransitionEvent()

    def engage_callback(self, msg):
        # print(msg)
        self.engage = msg

    def transition_callback(self, msg):
        # print(msg.go/al_state.label)
        self.transition_event = msg

    def get_engage(self):
        return self.engage
    
    def get_transition_event(self):
        return self.transition_event

def setup_logger(logpath):
    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s][%(levelname)-8s] %(message)s',
        handlers=[
            logging.FileHandler(os.path.join(logpath, "debug.log")),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger("topics_test")

if __name__ == "__main__":

    # Arguments
    window_size = 1000
    testing_time_seconds = 30
    log_folder = "log"

    topics_to_test = [
        "/clock",
        "/initialpose",
        "/localization/kinematic_state",
        "/localization/acceleration",
        "/planning/mission_planning/checkpoint",
        "/planning/mission_planning/goal",
        "/vehicle/status/control_mode",
        "/vehicle/status/gear_status",
        "/vehicle/status/steering_status",
        "/vehicle/status/turn_indicators_status",
        "/vehicle/status/velocity_status",
        "/perception/object_recognition/detection/objects",
        "/perception/object_recognition/ground_truth/objects",
        "/perception/obstacle_segmentation/pointcloud",
        "/perception/occupancy_grid_map/map"
    ]

    # Init
    rclpy.init()
    node = testNode()
    rt_hz = ROSTopicHz(node, window_size)
    logpath = os.path.join(log_folder, "latest")
    shutil.rmtree(logpath)
    Path(logpath).mkdir(parents=True, exist_ok=True)

    logger = setup_logger(logpath)

    # Wait for ss2
    logger.info("Wait for ss2")
    while node.get_engage().engage != True or node.get_transition_event().goal_state.label != "active":
        rclpy.spin_once(node)

    logger.info("Create subscribers")
    for topic in topics_to_test:
        msg_class = get_msg_class(node, topic, blocking=True, include_hidden_topics=True)

        if msg_class is None:
            continue

        node.create_subscription(
            msg_class,
            topic,
            functools.partial(rt_hz.callback_hz, topic=topic),
            qos_profile_sensor_data)

    test_result = {elem : [] for elem in topics_to_test}
    logger.info(f"Start logging until ss2 transition event active")
    start = time.time()
    end = time.time()
    while node.get_transition_event().goal_state.label == "active":
        for topic in test_result.keys():
            ret = rt_hz.get_hz(topic)
            if ret is not None:
                rate, min_delta, max_delta, std_dev, window = ret
                test_result[topic].append(rate * 1e9)

        rclpy.spin_once(node)
        end = time.time()
    logger.info("End logging")


    logger.info("Save to file")
    df = pd.DataFrame.from_dict(dict([ (k,pd.Series(v, dtype='float64')) for k,v in test_result.items() ]))
    df.to_csv (os.path.join(logpath, f'topics_hz.csv'), index = False, header=True)
    
    logger.info("Terminate test")
    shutil.copytree(logpath, os.path.join(log_folder, datetime.now().strftime("%d-%m-%Y-%H-%M-%S")))
    node.destroy_node()
    rclpy.shutdown()
