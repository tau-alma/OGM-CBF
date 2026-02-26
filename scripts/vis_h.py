#!/usr/bin/env python3

import os
import sys
from pathlib import Path

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
from mir_scenario import EeveeScenario

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
        #
SCENARIOS_EEVEE_THU = [
        EeveeScenario(EeveeScenario.ENV_THURSDAY,0),
        EeveeScenario(EeveeScenario.ENV_THURSDAY,1),
        EeveeScenario(EeveeScenario.ENV_THURSDAY,2),
        ]

TOPICS_MIR = [
        '/cbf/level_0',
        '/cbf/level_1',
        '/cbf/level_2',
        ]

TOPICS_EEVEE = [
        '/cbf/debug',
        ]

DOMINANT_RUN = 0

def vis_scenario_set(scenarios, name, show_x_lbl=True):

    topics = []
    series = {}
    
    for sc in scenarios:
        series[sc.name] = {}

        if type(sc) == MirScenario:
            for t in TOPICS_MIR:
                if t not in topics:
                    topics.append(t)
                d = 2
                series[sc.name][t] = []
                for c, ts, msg in sc.walk_topics([t]):
                    entry = np.array([ts, msg.data])
                    series[sc.name][t].append(entry)
                series[sc.name][t] = np.vstack(series[sc.name][t])
        
        elif type(sc) == EeveeScenario:
            for t in TOPICS_EEVEE:
                d = 2
                for c, ts, msg in sc.walk_topics([t]):
                    for hlvl, hval in enumerate(msg.h):
                        subt = "%s/level_%d" % (t, hlvl)
                        if hlvl >= len(msg.h)/2:
                            continue
                        if subt not in topics:
                            topics.append(subt)
                        if subt not in series[sc.name]:
                            series[sc.name][subt] = []
                        entry = np.array([ts, hval])
                        series[sc.name][subt].append(entry)
                for subt in series[sc.name].keys():
                    series[sc.name][subt] = np.vstack(series[sc.name][subt])

    fig = plt.figure(figsize=(10,3))
    ax = plt.gca()

    for sc in scenarios: 

        ts0 = sc.OFFSETS[sc.name]

        plot_kwargs = {
                }

        for t in topics:
            if 'level_0' in t:
                plot_kwargs['linestyle']='-'
                plot_kwargs['c']='red'
                lbl = ""
                #lbl += "Clutter: "
                lbl += "Level 0"
            elif 'level_1' in t:
                plot_kwargs['linestyle']='--'
                plot_kwargs['c']='green'
                lbl = ""
                #lbl += "Clutter: "
                lbl += "Level 1"
            elif 'level_2' in t:
                plot_kwargs['linestyle']=':'
                plot_kwargs['c']='blue'
                lbl = ""
                #lbl += "Clutter: "
                lbl += "Level 2"
            else:
                continue
            if sc.run == DOMINANT_RUN and not 'lr20' in sc.name:
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
    if type(sc) == MirScenario:
        ax.set_ylim((0.,2.5))    
        ax.set_xlim((0,50.))    
    elif type(sc) == EeveeScenario:
        ax.set_ylim((0.,6.5))    
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
#vis_scenario_set(
#        SCENARIOS_CLUTTER_SHORT + SCENARIOS_CLUTTER_LONG 
#        + SCENARIOS_CONCAVE_SHORT + SCENARIOS_CONCAVE_LONG,
#        "mir_all_cbf_h")    

vis_scenario_set(
        SCENARIOS_EEVEE_THU,
        "eevee_thu_cbf_h",
        show_x_lbl=True)    
