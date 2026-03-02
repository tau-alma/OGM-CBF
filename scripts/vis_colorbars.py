#!/usr/bin/env python3

import os
import sys
from pathlib import Path
import PIL
import cv2

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
import matplotlib.patches as patches
import matplotlib as mpl


CLRS = ['red', 'green', 'blue']

SDF_SCALE = 2

def vis_colorbar_sdf(name, max_val):

    fig = plt.figure(figsize=(4,0.8))
    ax = fig.subplots(1,1)

    c_map_ax = fig.add_axes([0.1, 0.8, 0.8, 0.1])

    cmap = plt.get_cmap('jet_r')      
    norm = mpl.colors.Normalize(vmin=0, vmax=max_val)

    cbar = plt.colorbar(
            mpl.cm.ScalarMappable(norm=norm, cmap=cmap),
            cax=c_map_ax,
            ticks=[0, max_val/2, max_val],
            orientation = 'horizontal',
            label = r'$\phi$(.) [m]',
            )

    cbar.outline.set_visible(False)

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])



    plt.savefig(name,
               bbox_inches='tight', 
               transparent=True,
               pad_inches=0)

def vis_colorbar_trav(name):

    fig = plt.figure(figsize=(4,0.8))
    ax = fig.subplots(1,1)

    c_map_ax = fig.add_axes([0.1, 0.8, 0.8, 0.1])

    cmap = mpl.colors.ListedColormap(
            [(4/255,133/255,15/255),(225/255,138/255,5/255),(243/255,1/255,4/255)],
            name='trav',
            N=None)
    norm = mpl.colors.Normalize(vmin=0, vmax=3)

    cbar = plt.colorbar(
            mpl.cm.ScalarMappable(norm=norm, cmap=cmap),
            cax=c_map_ax,
            ticks=[],
            orientation = 'horizontal',
            label = r'Occupancy',
            )

    #cbar.set_ticks([0.5,1.5,2.5],labels=["Free", "Inflated", "Occupied"],rotation=20)
    cbar.set_ticks([0.5,1.5,2.5],labels=["Free", "Inflated", "Occupied"])
    #cbar.set_ticklabels(["Free", "Inflated", "Occupied"])

    cbar.outline.set_visible(False)

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])



    plt.savefig(name,
               bbox_inches='tight', 
               transparent=True,
               pad_inches=0)

vis_colorbar_sdf(
        name="/tmp/colorbar_sdf_5.pdf",
        max_val=5.0,
        )
vis_colorbar_trav(
        name="/tmp/colorbar_trav.pdf",
        )

