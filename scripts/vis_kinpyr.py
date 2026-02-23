#!/usr/bin/env python3

import os
import sys
from pathlib import Path
import PIL

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import rc
plt.rcParams['ps.useafm'] = True
rc('font',**{
    'family':'serif',
    'size' : 16,
    })
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['text.usetex'] = True
plt.rcParams['text.latex.preamble']=r"\usepackage{amsmath}"


def vis_traj(pth_traj, pth_map):

    raw = np.loadtxt(pth_traj)


    ls = (raw.shape[1]-4-2)//2


    traj_xy = raw[:,:2]
    traj_uv = raw[:,2:4]
    sdf_pt = raw[:,4:6]
    sdf_grad_levels =raw[:,6:] 

    plt.figure(figsize=(10,10))
    ax = plt.gca()

    with PIL.Image.open(pth_map) as img:
        ax.imshow(img, cmap='gray')

    idx = [0, 20, 40, 60, 80]

    kwargs = {
            "width" : 0.005,
    #        "headwidth" : 5.,
            }

    plt.plot(
            traj_xy[idx[0]:idx[-1],0],traj_xy[idx[0]:idx[-1],1],
            c='blue', label="Trajectory", linewidth=3)

    ax.quiver(
        traj_xy[idx,0], traj_xy[idx,1], traj_uv[idx,0], traj_uv[idx,1],
        angles="xy",
        color='blue',
        #label="Heading",
        **kwargs)

    clrs = ['cyan', 'magenta', 'orange']
    for l in range(ls):
        ax.quiver(
            sdf_pt[idx,0], sdf_pt[idx,1],
            sdf_grad_levels[idx,l*2], sdf_grad_levels[idx,l*2+1],
            #scale = scales[level],
            color=clrs[l],
            label=r"$\nabla\phi'$ Level $%d$" % (l+1),
            **kwargs)


    leg = ax.legend(ncol=2, loc='upper center',  bbox_to_anchor=(0.4, 0.9))    
    leg.get_frame().set_alpha(0)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])
    plt.savefig("/tmp/kinpyr_traj_s8.pdf",
               bbox_inches='tight', 
               transparent=True,
               pad_inches=0)

vis_traj(
        "/data/repos/ogm-cbf/ogm-cbf-mir/scripts/kinpyr_traj_s8.txt",
        "/data/repos/ogm-cbf/ogm-cbf-pyramid/ros2_ws/src/ogm_cbf_kinematic_sim/ogm_cbf_kinematic_sim/map/pyramid/Frame_22_2.png")    
