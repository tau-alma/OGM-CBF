#!/usr/bin/env python3

import os
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

BAG_ROOT="/data/data/mir/20260208_mir_festia_clean"


def process_entry(msg, connection, timestamp):
    asf = timestamp / 1e9
    #print(asf, connection.msgtype)
    entry = np.array([asf, msg.data])
    return entry

class MirScenario:

    ENV_CLUTTER = 'clutter'
    ENV_CONCAVE = 'concave'

    SENSOR_SHORT = "sr5"
    SENSOR_LONG = "lr20"

    OFFSETS = {
            "concave_sr5_00" : 1770578242.5245247,
            "concave_sr5_01" : 1770578242.5245247 + 3407.5,
            "concave_sr5_02" : 1770578242.5245247 + 3587.,
            "concave_sr5_03" : 1770578242.5245247 + 3805.5,
            "concave_sr5_04" : 1770578242.5245247 + 4011.,
            # 
            "concave_lr20_00" : 1770579347.0245247,
            "concave_lr20_01" : 1770579347.0245247 + 634.4,
            "concave_lr20_02" : 1770579347.0245247 + 782.5,
            "concave_lr20_03" : 1770579347.0245247 + 1832.8,
            "concave_lr20_04" : 1770579347.0245247 + 2003.2,
            # 
            "clutter_sr5_00" : 1770578790.5245247,
            "clutter_sr5_01" : 1770578790.5245247 + 127.,
            "clutter_sr5_02" : 1770578790.5245247 + 243.6,
            "clutter_sr5_03" : 1770578790.5245247 + 373.5,
            "clutter_sr5_04" : 1770578790.5245247 + 728.,
            # 
            "clutter_lr20_00" : 1770579694.1245247,
            "clutter_lr20_01" : 1770579694.1245247 + 128.3,
            "clutter_lr20_02" : 1770579694.1245247 + 242.2,
            "clutter_lr20_03" : 1770579694.1245247 + 353.5,
            "clutter_lr20_04" : 1770579694.1245247 + 695.9,
            }

    PTHS = {
        (ENV_CLUTTER, SENSOR_SHORT) :
        [
            "clutter_sr5/rosbag2_2026_02_08-21_26_23",
            "clutter_sr5/rosbag2_2026_02_08-21_28_28",
            "clutter_sr5/rosbag2_2026_02_08-21_30_25",
            "clutter_sr5/rosbag2_2026_02_08-21_32_37",
            "clutter_sr5/rosbag2_2026_02_08-21_38_31",
           ],
        (ENV_CLUTTER, SENSOR_LONG) :
        [
            "clutter_lr20/rosbag2_2026_02_08-21_41_27",
            "clutter_lr20/rosbag2_2026_02_08-21_43_35",
            "clutter_lr20/rosbag2_2026_02_08-21_45_27",
            "clutter_lr20/rosbag2_2026_02_08-21_47_20",
            "clutter_lr20/rosbag2_2026_02_08-21_53_01",
           ],
        (ENV_CONCAVE, SENSOR_SHORT) :
        [
            "concave_sr5/rosbag2_2026_02_08-21_17_14",
            "concave_sr5/rosbag2_2026_02_08-22_13_59",
            "concave_sr5/rosbag2_2026_02_08-22_17_00",
            "concave_sr5/rosbag2_2026_02_08-22_20_36",
            "concave_sr5/rosbag2_2026_02_08-22_24_00",
            ],
        (ENV_CONCAVE, SENSOR_LONG) :
        [
            "concave_lr20/rosbag2_2026_02_08-21_35_41",
            "concave_lr20/rosbag2_2026_02_08-21_46_17",
            "concave_lr20/rosbag2_2026_02_08-21_48_40",
            "concave_lr20/rosbag2_2026_02_08-22_06_11",
            "concave_lr20/rosbag2_2026_02_08-22_09_00",
            ]
        }

    def get_path(self):
        relative = self.PTHS[(self.environment, self.sensor)][self.run]
        return os.path.join(BAG_ROOT, relative)

    def get_name(self):
        name = "%s_%s_%02d" % (
                self.environment,
                self.sensor,
                self.run)
        return name

    def walk_topics(self, topics):

        bagpath = Path(self.bag_path)
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        with AnyReader([bagpath], default_typestore=typestore) as reader:
            connections = [x for x in reader.connections if x.topic in topics]

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                yield (connection, timestamp/1e9, msg)
    
    def __init__(self, environment, sensor, run):

        self.environment = environment
        self.sensor = sensor
        self.run = run

        self.name = self.get_name()
        self.bag_path = self.get_path()


