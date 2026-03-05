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
    'size' : 15,
    })
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['text.usetex'] = True
plt.rcParams['text.latex.preamble']=r"\usepackage{amsmath}"
import matplotlib.patches as patches
import matplotlib as mpl



SDF_SCALE = 11 #7 
SDF_SCALE_NEG = 220
QUIVER_STEP = 50
THR_OCC = 0.66

def vis_sdf(pth_map, name):

    img = 255 - cv2.imread(pth_map, cv2.IMREAD_GRAYSCALE)
    h, w = img.shape

    img[img > THR_OCC * 255] = 255
    img[img <= THR_OCC * 255] = 0
    
    sdf = cv2.distanceTransform(255 - img, cv2.DIST_L2, 3, dstType=cv2.CV_32F)
    sdf_inv = cv2.distanceTransform(img, cv2.DIST_L2, 3, dstType=cv2.CV_32F)
    sdf = sdf - sdf_inv
    nabla = np.gradient(sdf)

    print (np.min(sdf), np.max(sdf))

    max_val = np.ceil(max(h,w) / SDF_SCALE)
    min_val = -1*np.ceil(max(h,w) / SDF_SCALE_NEG)
    x = np.copy(sdf)
    x[x > max_val] = max_val
    x[x < min_val] = min_val
    print (np.min(x), np.max(x))

    x = ( (x-min_val) / (max_val-min_val) * 255).astype(np.uint8)
    x = cv2.applyColorMap(x, cv2.COLORMAP_JET)
    

    fig = plt.figure(figsize=(10,2.3))
    ax = fig.subplots(1,1)

    #ax.imshow(img)
    ax.imshow(x)

    vecs = []
    for i in range(QUIVER_STEP//2,h,QUIVER_STEP):
        for j in range(QUIVER_STEP//2,w,QUIVER_STEP):
            vecs.append([i,j,nabla[0][i,j],nabla[1][i,j]])
            #print (sdf[i,j],np.sqrt(np.power(nabla[0][i,j],2)+np.power(nabla[1][i,j],2)))
    vecs = np.array(vecs)
    #print (vecs.shape)

    kwargs = {
            "width" : 0.003,
            "headwidth" : 3.,
            "headaxislength" : 3.,
            "headlength" : 3.,
            }

    ax.quiver(
        vecs[:,1], vecs[:,0], vecs[:,3], vecs[:,2],
        angles="xy",
        color='black',
        scale = 80,
        **kwargs,
        )

    print (h,w)

    cmap = plt.get_cmap('jet_r')      
    norm = mpl.colors.Normalize(vmin=min_val, vmax=max_val)

    c_map_ax = fig.add_axes([.91, 0.2, 0.03, 0.6])

    cbar = plt.colorbar(
            mpl.cm.ScalarMappable(norm=norm, cmap=cmap),
            cax=c_map_ax,
            #ticks=[0, max_val/2, max_val],
            ticks=range(0,int(max_val),100),
            orientation = 'vertical',
            label = r"$\phi'$(.) [px]",
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
               dpi=300,
               transparent=True,
               pad_inches=0)


IMG = "/home/mp/repos/papers/tau-papers/ogm-cbf/fig/carla_method_top_occ.jpg"
vis_sdf(IMG, "/tmp/carla_method_top_sdf.pdf")

