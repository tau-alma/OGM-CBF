#!/usr/bin/env python3

import os
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


import numpy as np
from matplotlib import pyplot as plt
from matplotlib import rc
plt.rcParams['ps.useafm'] = True
rc('font',**{
    'family':'serif',
    'size' : 20,
    })
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['text.usetex'] = True
plt.rcParams['text.latex.preamble']=r"\usepackage{amsmath}"

CARLA_COLLISION_EVENT_MSG = """
std_msgs/Header header
uint32 other_actor_id
geometry_msgs/Vector3 normal_impulse
"""

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
TYPESTORE.register(get_types_from_msg(CARLA_COLLISION_EVENT_MSG, 'carla_msgs/msg/CarlaCollisionEvent'))

TOPIC_COLLISION = '/carla/ego_vehicle/collision'
TOPIC_ODOM = '/carla/ego_vehicle/odometry'
TOPIC_VCBF_COLLISION = '/vcbf/collision'
TOPIC_VCBF_ODOM = '/vcbf/odom'

SCENARIOS = ['simple', 'trap']
MODELS = ['ogm','nomem','vcbf']

TOPICS = {
        'ogm' : [TOPIC_COLLISION, TOPIC_ODOM],
        'nomem' : [TOPIC_COLLISION, TOPIC_ODOM],
        'vcbf' : [TOPIC_VCBF_COLLISION, TOPIC_VCBF_ODOM],
        }


OBSTACLES_TRAP = np.array([
        [-41.0+1.25,-64.],
        [-45.5+1.25,-51.0],
        [-47.0+1.25,-59.0],
        ])
OBSTACLES_SIMPLE = np.array([
        [-38.0,-73.5],
        [-45.5,-85.0],
        [-42.5,-62.0],
        ])

OBSTACLES = {
        'trap' : OBSTACLES_TRAP,
        'simple' : OBSTACLES_SIMPLE,
        }

K_PTH = 'bag_pth'
K_SCENARIO = 'scenario'
K_MODEL = 'model'
K_COLLISION = 'collision'
K_MINDST = 'mindst'

def walk_topics(bag_path, topics):
    bagpath = Path(bag_path)
    print (bag_path)
    with AnyReader([bagpath], default_typestore=TYPESTORE) as reader:
        connections = [x for x in reader.connections if x.topic in topics]

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            yield (connection, timestamp/1e9, msg)

def process_run(bag_path, scenario, model):

    has_collision = False

    dsts = []

    obstacles = OBSTACLES[scenario]

    for c, ts, msg in walk_topics(bag_path, TOPICS[model]):
        if c.topic in [TOPIC_COLLISION, TOPIC_VCBF_COLLISION]:
            has_collision = True
        if c.topic in [TOPIC_ODOM, TOPIC_VCBF_ODOM]:
            pos = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y]])
            dst = np.linalg.norm(pos[np.newaxis,...] - obstacles[:,np.newaxis,...],axis=2)[...,0]
            dsts.append(dst)
 
    dsts = np.array(dsts)

    min_dst = np.min(dsts)

    return {K_COLLISION : has_collision, K_MINDST : min_dst}


def evaluate_run_set(run_set):
    for run in run_set:
        bag_pth = run[K_PTH]
        scenario = run[K_SCENARIO]
        model = run[K_MODEL]

        res = process_run(bag_pth, scenario, model)

        has_collision = res[K_COLLISION]
        min_dst = res[K_MINDST]

        for k,v in res.items():
            run[k] = v

        print ("%s in %s : collision %x, min dst %.2f" % (model, scenario, has_collision, min_dst) )

def tabulize_run_set(run_sets):

    tab = {}

    for scenario in SCENARIOS: 
        for model in MODELS:
            
            sucs = []
            dsts = []

            for r in run_sets:
                if r[K_SCENARIO] == scenario and r[K_MODEL] == model:
                    sucs.append(float(r[K_COLLISION]))
                    if not r[K_COLLISION]:
                        dsts.append(r[K_MINDST])

            tab[(scenario,model)] = {K_COLLISION : sucs, K_MINDST : dsts}        

    fancy_scenarios = {
            'trap' : 'Trap',
            'simple' : 'Open',
            }

    fancy_models = {
            'ogm' : 'OGM-CBF',
            'nomem' : 'MU-OGM-CBF',
            'vcbf' : 'VCBF',
            }


    def compose_title_line():
        
        ss = ""
        ss += "%25s" % "Model"
        for scenario in SCENARIOS:
            ss += " & \\multicolumn{2}{c}{%25s}" % fancy_scenarios[scenario]
        ss += " \\\\"    
        ss += "\n"    
        ss += "%25s" % ""
        for scenario in SCENARIOS:
            #ss += " & Success Rate & Min. Dist. to Obstacle $(\\mu\\pm\\sigma)$"
            ss += " & Succ. & $L2_\\text{min}$ $(\\mu\\pm\\sigma)$"
        ss += " \\\\"    
        ss += "\n"    
        
        return ss

    def compose_model_line(model):

        ss = ""
        ss += "%25s" % fancy_models[model]
        
        for scenario in SCENARIOS:

            entry = tab[(scenario,model)]

            success_rate = 100 - np.mean(entry[K_COLLISION])*100
            mu_dst = np.mean(entry[K_MINDST])
            std_dst = np.std(entry[K_MINDST])

            ss += " & $\\SI{%3.0f}{\percent}$" % success_rate
            ss += " & $\\num{%1.2f} \\pm \\SI{%1.2f}{\meter}$" % (mu_dst, std_dst)

        ss += " \\\\"    
        ss += "\n"    


        return ss

    sss = ""
    sss += "\\begin{tabular}{l" + "".join([" rr" for _ in SCENARIOS]) + "}\n"
    sss += "\\toprule\n"
    sss += compose_title_line()
    sss += "\\midrule\n"
    for model in MODELS:
        sss += compose_model_line(model)
    sss += "\\midrule\n"
    sss += "\\end{tabular}\n"

    print (sss)    

    with open('/tmp/carla_tbl.tex','w') as f:
        f.write (sss)



RUNS_TRAP_OGM = [
        {K_PTH : "/data/data/carla/trap/our_38_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_39_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_39_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_40_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_40_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_41_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_41_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_42_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_42_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_43_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_43_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_44_0/nogm_44_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_44_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_45_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_45_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_46_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_46_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_47_0", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/trap/our_47_5", K_SCENARIO : 'trap', K_MODEL : 'ogm'},
        ]

RUNS_TRAP_NOMEM = [
        {K_PTH : "/data/data/carla/trap/nomem_38_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_39_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_39_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_40_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_40_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_41_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_41_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_42_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_42_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_43_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_43_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_44_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_44_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_45_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_45_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_46_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_46_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_47_0", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/trap/nomem_47_5", K_SCENARIO : 'trap', K_MODEL : 'nomem'},
        ]

RUNS_TRAP_VCBF = [
        {K_PTH : "/data/data/carla/trap/vcbf_38_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_39_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_39_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_40_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_40_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_41_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_41_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_42_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_42_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_43_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_43_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_44_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_44_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_45_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_45_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_46_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_46_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_47_0", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/trap/vcbf_47_5", K_SCENARIO : 'trap', K_MODEL : 'vcbf'},
        ]

RUNS_SIMPLE_OGM = [
        {K_PTH : "/data/data/carla/simple/our_38_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_39_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_39_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_40_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_40_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_41_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_41_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_42_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_42_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_43_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_43_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_44_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_44_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_45_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_45_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_46_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_46_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_47_0", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        {K_PTH : "/data/data/carla/simple/our_47_5", K_SCENARIO : 'simple', K_MODEL : 'ogm'},
        ]

RUNS_SIMPLE_NOMEM = [
        {K_PTH : "/data/data/carla/simple/nomem_38_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_39_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_39_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_40_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_40_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_41_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_41_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_42_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_42_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_43_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_43_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_44_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_44_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_45_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_45_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_46_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_46_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_47_0", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        {K_PTH : "/data/data/carla/simple/nomem_47_5", K_SCENARIO : 'simple', K_MODEL : 'nomem'},
        ]

RUNS_SIMPLE_VCBF = [
        {K_PTH : "/data/data/carla/simple/vcbf_38_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_39_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_39_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_40_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_40_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_41_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_41_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_42_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_42_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_43_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_43_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_44_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_44_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_45_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_45_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_46_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_46_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_47_0", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        {K_PTH : "/data/data/carla/simple/vcbf_47_5", K_SCENARIO : 'simple', K_MODEL : 'vcbf'},
        ]
RUNS = RUNS_TRAP_OGM + RUNS_TRAP_NOMEM + RUNS_TRAP_VCBF + RUNS_SIMPLE_OGM + RUNS_SIMPLE_NOMEM + RUNS_SIMPLE_VCBF

evaluate_run_set(RUNS)
tabulize_run_set(RUNS)
