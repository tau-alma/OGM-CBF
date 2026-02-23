#!/usr/bin/env python3

import os
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

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

from mir_scenario import MirScenario

SCENARIOS_CLUTTER_SHORT = [
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_SHORT,0),
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_SHORT,1),
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_SHORT,2),
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_SHORT,3),
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_SHORT,4),
        ]
        #
SCENARIOS_CLUTTER_LONG = [
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_LONG,0),
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_LONG,1),
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_LONG,2),
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_LONG,3),
        MirScenario(MirScenario.ENV_CLUTTER, MirScenario.SENSOR_LONG,4),
        ]
        #
SCENARIOS_CONCAVE_SHORT = [
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_SHORT,0),
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_SHORT,1),
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_SHORT,2),
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_SHORT,3),
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_SHORT,4),
        ]
        #
SCENARIOS_CONCAVE_LONG = [
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_LONG,0),
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_LONG,1),
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_LONG,2),
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_LONG,3),
        MirScenario(MirScenario.ENV_CONCAVE, MirScenario.SENSOR_LONG,4),
        ]

TOPICS = [
        '/cbf/level_0',
        '/cbf/level_1',
        '/cbf/level_2',
        ]

DOMINANT_RUN = 0

def vis_scenario_set(scenarios, name, show_x_lbl=True):

    series = {}
    for sc in scenarios:
        series[sc.name] = {}
        for t in TOPICS:
            d = 2
            series[sc.name][t] = []
            for c, ts, msg in sc.walk_topics([t]):
                entry = np.array([ts, msg.data])
                series[sc.name][t].append(entry)
            series[sc.name][t] = np.vstack(series[sc.name][t])


    fig = plt.figure(figsize=(10,3))
    ax = plt.gca()

    for sc in scenarios: 

        ts0 = sc.OFFSETS[sc.name]

        plot_kwargs = {
                }

        for t in TOPICS:
            if 'clutter' in sc.name and 'level_0' in t:
                plot_kwargs['linestyle']='-'
                plot_kwargs['c']='red'
                lbl = ""
                #lbl += "Clutter: "
                lbl += "Level 0"
            elif 'clutter' in sc.name and 'level_1' in t:
                plot_kwargs['linestyle']='--'
                plot_kwargs['c']='green'
                lbl = ""
                #lbl += "Clutter: "
                lbl += "Level 1"
            elif 'clutter' in sc.name and 'level_2' in t:
                plot_kwargs['linestyle']=':'
                plot_kwargs['c']='blue'
                lbl = ""
                #lbl += "Clutter: "
                lbl += "Level 2"
            elif 'concave' in sc.name and 'level_0' in t:
                plot_kwargs['linestyle']='-'
                plot_kwargs['c']='cyan'
                lbl = ""
                #lbl += "Concave: "
                lbl += "Level 0"
            elif 'concave' in sc.name and 'level_1' in t:
                plot_kwargs['linestyle']='--'
                plot_kwargs['c']='magenta'
                lbl = ""
                #lbl += "Concave: "
                lbl += "Level 1"
            elif 'concave' in sc.name and 'level_2' in t:
                plot_kwargs['linestyle']=':'
                plot_kwargs['c']='orange'
                lbl = ""
                #lbl += "Concave: "
                lbl += "Level 2"
            else:
                assert False
            if sc.run == DOMINANT_RUN and 'sr5' in sc.name:
                plot_kwargs['label']=lbl
                plot_kwargs['alpha']=1.
                plot_kwargs['linewidth']=2.
            else:
                plot_kwargs['alpha']=.1
                plot_kwargs['linewidth']=1.5
            
            plt.plot(
                    series[sc.name][t][:,0]-ts0,
                    series[sc.name][t][:,1],
                    **plot_kwargs)

    #ax.set_yscale('log')    
    #ax.set_ylim((10**(-1.5),10**(0.5)))    
    ax.set_ylim((0.,2.5))    
    ax.set_xlim((0,50.))    

    if show_x_lbl:
        ax.set_xlabel("Time [s]")    
    else:
        ax.set_xticklabels([])

    ax.set_ylabel(r"$h(\boldsymbol{x})$ [--]")    
    leg = ax.legend(ncols=1)    
    leg.get_frame().set_alpha(0)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    plt.savefig("/tmp/%s.pdf" % name,
               bbox_inches='tight', 
               transparent=True,
               pad_inches=0)

vis_scenario_set(
        SCENARIOS_CLUTTER_SHORT + SCENARIOS_CLUTTER_LONG,
        "mir_clutter_cbf_h",
        show_x_lbl=False)    
vis_scenario_set(
        SCENARIOS_CONCAVE_SHORT + SCENARIOS_CONCAVE_LONG,
        "mir_concave_cbf_h")    
vis_scenario_set(
        SCENARIOS_CLUTTER_SHORT + SCENARIOS_CLUTTER_LONG 
        + SCENARIOS_CONCAVE_SHORT + SCENARIOS_CONCAVE_LONG,
        "mir_all_cbf_h")    
