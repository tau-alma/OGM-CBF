#!/usr/bin/env python3

import os
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

CBF_DEBUG_MSG = """
std_msgs/Header header
float64 b_ref
bool feasible
string[] name
float64[] h
float64[] o_o
float64[] eta
float64[] distance
float64[] radius
"""


TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
TYPESTORE.register(get_types_from_msg(CBF_DEBUG_MSG, 'cbf_msgs/msg/CbfDebug'))

def process_entry(msg, connection, timestamp):
    asf = timestamp / 1e9
    #print(asf, connection.msgtype)
    entry = np.array([asf, msg.data])
    return entry

class Scenario:

    def get_path(self):
        assert False, "not implemented"

    def get_name(self):
        assert False, "not implemented"

    def walk_topics(self, topics):

        bagpath = Path(self.bag_path)

        with AnyReader([bagpath], default_typestore=TYPESTORE) as reader:
            connections = [x for x in reader.connections if x.topic in topics]

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                yield (connection, timestamp/1e9, msg)

    def __init__(self):
        pass

class MirScenario(Scenario):

    BAG_ROOT="/data/data/mir/20260208_mir_festia_clean"

    ENV_CLUTTER = 'clutter'
    ENV_CONCAVE = 'concave'

    SENSOR_SHORT = "sr5"
    SENSOR_LONG = "lr20"

    OFFSETS = {
            "concave_lr20_00" : 1770578242.5245247,
            "concave_lr20_01" : 1770578242.5245247 + 3407.5,
            "concave_lr20_02" : 1770578242.5245247 + 3587.,
            "concave_lr20_03" : 1770578242.5245247 + 3805.5,
            "concave_lr20_04" : 1770578242.5245247 + 4011.,
            # 
            "concave_sr5_00" : 1770579347.0245247,
            "concave_sr5_01" : 1770579347.0245247 + 634.4,
            "concave_sr5_02" : 1770579347.0245247 + 782.5,
            "concave_sr5_03" : 1770579347.0245247 + 1832.8,
            "concave_sr5_04" : 1770579347.0245247 + 2003.2,
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
            "concave_sr5/rosbag2_2026_02_08-21_35_41",
            "concave_sr5/rosbag2_2026_02_08-21_46_17",
            "concave_sr5/rosbag2_2026_02_08-21_48_40",
            "concave_sr5/rosbag2_2026_02_08-22_06_11",
            "concave_sr5/rosbag2_2026_02_08-22_09_00",
            ],
        (ENV_CONCAVE, SENSOR_LONG) :
        [
            "concave_lr20/rosbag2_2026_02_08-21_17_14",
            "concave_lr20/rosbag2_2026_02_08-22_13_59",
            "concave_lr20/rosbag2_2026_02_08-22_17_00",
            "concave_lr20/rosbag2_2026_02_08-22_20_36",
            "concave_lr20/rosbag2_2026_02_08-22_24_00",
            ]
        }

    def get_path(self):
        relative = self.PTHS[(self.environment, self.sensor)][self.run]
        return os.path.join(self.BAG_ROOT, relative)

    def get_name(self):
        name = "%s_%s_%02d" % (
                self.environment,
                self.sensor,
                self.run)
        return name
    
    def __init__(self, environment, sensor, run):

        self.environment = environment
        self.sensor = sensor
        self.run = run

        self.name = self.get_name()
        self.bag_path = self.get_path()


class EeveeScenario(Scenario):
    
    BAG_ROOT="/data/data/eevee/cbf"

    ENV_THURSDAY = 'thursday'

    OFFSETS = {
            "thursday_00" : 1772118039.286425,
            "thursday_01" : 1772118039.286425+292.,
            "thursday_02" : 1772118039.286425+705.,
            }

    PTHS = {
        (ENV_THURSDAY) :
        [
            "thursday_cbf/front_la_15_R2_a015_gausblur_wref_cont_2_obst_2_10_02_linvel_gaus_8_6_kw_052_1_kb_12_3_map7pyr3__2_recording",
            "thursday_cbf/front_la_15_R2_a015_gausblur_wref_cont_2_obst_2_10_02_linvel_gaus_8_6_kw_052_1_kb_12_3_map7pyr3__2_recording_2",
            "thursday_cbf/front_la_15_R2_a015_gausblur_wref_cont_2_obst_2_10_02_linvel_gaus_8_6_kw_052_1_kb_12_3_map7pyr3__2_recording_3",
            ]
        }

    def get_path(self):
        relative = self.PTHS[(self.environment)][self.run]
        return os.path.join(self.BAG_ROOT, relative)

    def get_name(self):
        name = "%s_%02d" % (
                self.environment,
                self.run)
        return name

    def __init__(self, environment, run):

        self.environment = environment
        self.run = run

        self.name = self.get_name()
        self.bag_path = self.get_path()

