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
    'size' : 11,
    })
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['text.usetex'] = True
plt.rcParams['text.latex.preamble']=r"\usepackage{amsmath}"
import matplotlib.patches as patches
import matplotlib as mpl


CLRS = ['red', 'green', 'blue']

SDF_SCALE = 2

def get_background(img, pyr_level, pyr_sigma):

    h, w = img.shape

    x = cv2.distanceTransform(img, cv2.DIST_L2, 3, dstType=cv2.CV_32F)
    for k in range(1, pyr_level):
        
        if pyr_sigma > 0: 
            x = cv2.GaussianBlur(x, (0, 0), sigmaX=pyr_sigma, sigmaY=pyr_sigma)
        x = cv2.pyrDown(x)              # Gaussian+downsample on float32 meters

    x = cv2.resize(x, (h,w))
    
    x = (x / max(h,w) * SDF_SCALE * 255).astype(np.uint8)
    x = cv2.applyColorMap(x, cv2.COLORMAP_JET)
    x[img < 127, :] = 0 

    return x    

def vis_legend(sfig, **kwargs):

    if 'pyr_level' in kwargs:
        pyr_level = kwargs['pyr_level']
    else:
        pyr_level = 0

    if 'vis_legend' in kwargs:
        vis_legend = kwargs['vis_legend']
    else:
        vis_legend = False
   
    if not vis_legend:
        return

    ax = sfig.subplots(1,1)


    plts = []
    
    plts.append(ax.plot([0,1],[0, 0],
            color="gray",
            label=r"Trajectory",
            ))
    plts.append(ax.plot([0,1],[0, 0],
            color="white",
            label=r"with Headings",
            ))
    ax.quiver(
        [0.38], [0.9], [-1], [0],
        angles="xy",
        color='gray',
        width=0.02,
        scale=5,
        )


    for l in range(pyr_level):
        plts.append(ax.plot([0,1],[0, 0],
                color=CLRS[l%len(CLRS)],
                label=r"$\nabla\phi'_{%d}$" % (l+1),
                ))
        ax.quiver(
            [0.38], [0.675-l*0.12], [-1], [0],
            angles="xy",
            color=CLRS[l%len(CLRS)],
            width=0.02,
            scale=5,
            )

    leg = ax.legend(ncol=1)    
    leg.get_frame().set_alpha(0)
    
    for p in plts:    
        for l in p:
            l.set_visible(False)
   
    ax.quiver(
        [0.9], [0.65], [0], [-1],
        angles="xy",
        color='black',
        width=0.02,
        scale=1.55,
        )
    ax.text(0.8, 0.0,
            r"Target Heading $\psi_\text{ref}$",
            rotation = 90)
    ax.set_xlim((0, 1))
    ax.set_ylim((0, 1))


    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)

    ax.set_xticks([])
    ax.set_yticks([])

def vis_colorbar(sfig, pth_map, **kwargs):

    if 'vis_colorbar' in kwargs:
        vis_colorbar = kwargs['vis_colorbar']
    else:
        vis_colorbar = False
   
    if not vis_colorbar:
        return

    ax = sfig.subplots(1,1)

    img = cv2.imread(pth_map, cv2.IMREAD_GRAYSCALE)
    h, w = img.shape

    c_map_ax = sfig.add_axes([0.1, 0.8, 0.8, 0.1])

    cmap = plt.get_cmap('jet_r')      
    norm = mpl.colors.Normalize(vmin=0, vmax=max(h,w)/SDF_SCALE)

    cbar = plt.colorbar(
            mpl.cm.ScalarMappable(norm=norm, cmap=cmap),
            cax=c_map_ax,
            #ticks=[0.5],
            orientation = 'horizontal',
            label = r"$\phi'$(.) [cell]",
            )

    print (cbar.__dict__)
    cbar.outline.set_visible(False)

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])


def vis_traj(sfig, pth_traj, pth_map, **kwargs):

    name = pth_traj.split('/')[-1].split('.')[0]
    raw = np.loadtxt(pth_traj)
    ls = (raw.shape[1]-4-2)//2

    #traj_xy = raw[:,:2]
    #traj_uv = raw[:,2:4]
    #sdf_pt = raw[:,4:6]
    #sdf_grad_levels =raw[:,6:] 
    traj_xy = raw[:,1:3]
    traj_yaw = raw[:,3:4]
    traj_uv = np.concatenate([
        1.0 * np.cos(traj_yaw),
        -1.0 * np.sin(traj_yaw)
        ],axis=1)
    sdf_pt = raw[:,5:7]
    sdf_grad_levels =raw[:,7:] 

    if 'pyr_level' in kwargs:
        pyr_level = kwargs['pyr_level']
    else:
        pyr_level = 0

    if 'pyr_sigma' in kwargs:
        pyr_sigma = kwargs['pyr_sigma']
    else:
        pyr_sigma = -1


    ax = sfig.subplots(1,1)

    img = cv2.imread(pth_map, cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    
    img = get_background(img, pyr_level, pyr_sigma)

    _im = ax.imshow(
            img,
            #norm=mpl.colors.Normalize(vmin=0, vmax=255, clip=True),
            #extent=(-.5+clip[0]*w, w-.5-clip[1]*w, h-.5-clip[2]*h,-.5+clip[3]*h)
            )
    #_patch = patches.Circle((260, 200), radius=20, transform=ax.transData)
    #_im.set_clip_path(_patch)

    #idx = [0, 20, 40, 60, 80]
    idx = range(0,traj_xy.shape[0],20)

    kwargs = {
            "width" : 0.01,
            "headwidth" : 5.,
            }

    ax.plot(
            traj_xy[idx[0]:idx[-1],0],traj_xy[idx[0]:idx[-1],1],
            c='gray', label="Trajectory with Headings", linewidth=2)


    ax.quiver(
        traj_xy[idx,0], traj_xy[idx,1], traj_uv[idx,0], traj_uv[idx,1],
        angles="xy",
        color='gray',
        #scale = 10,
        #label="Heading",
        **kwargs)

    for l in range(ls):
        ax.quiver(
            sdf_pt[idx,0], sdf_pt[idx,1],
            sdf_grad_levels[idx,l*2], -1*sdf_grad_levels[idx,l*2+1],
            angles="xy",
            scale = 10,
            color=CLRS[l%len(CLRS)],
            label=r"$\nabla\phi'_{%d}$" % (l+1),
            **kwargs)


    if pyr_sigma > 0:    
        #ax.set_title(r"Pyramid Level $%d$, $\sigma=%.1f$" % (pyr_level, pyr_sigma))
        sigma_true = np.sqrt(pyr_sigma*pyr_sigma + 1) 
        ax.set_title(r"$\phi'_{%d},\ \sigma=%.1f$" % (pyr_level, pyr_sigma))
    elif pyr_level > 1:    
        #ax.set_title(r"Pyramid Level $%d$, $\sigma=%.1f$" % (pyr_level, pyr_sigma))
        ax.set_title(r"$\phi'_{%d},\ \sigma=%.1f$" % (pyr_level, 1.0))
    else:
        ax.set_title(r"$\phi'_{%d}$" % (pyr_level))

    #leg = ax.legend(ncol=2, loc='upper center',  bbox_to_anchor=(0.4, 0.9))    
    #leg.get_frame().set_alpha(0)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])

    ax.set_xlim((50,300))
    ax.set_ylim((50,300))
    
    #plt.savefig("/tmp/%s.pdf" % name,
    #           bbox_inches='tight', 
    #           transparent=True,
    #           pad_inches=0)


def vis_traj_set(traj_set):
    
    stsz = len(traj_set)
    w = 3
    h = (stsz // w) 
    if stsz % w:
        h += 1

    #fig = plt.figure(layout='tight', figsize=(10*w,10*h))
    fig = plt.figure(layout='constrained',figsize=(2*w,2*h))
    subfigs = fig.subfigures(h,w,hspace=.0,wspace=0.)

    for ij, entry in enumerate(traj_set):
        i = ij // w
        j = ij % w
        traj_pth, img_pth, kwargs = entry
        if traj_pth is not None:
            vis_traj(subfigs[i, j], traj_pth, img_pth, **kwargs)
        else:
            vis_legend(subfigs[i, j], **kwargs)
            vis_colorbar(subfigs[i, j], img_pth, **kwargs)

    plt.savefig("/tmp/kinset.pdf",
               bbox_inches='tight', 
               transparent=True,
               dpi=60,
               pad_inches=0)

IMG = "/data/repos/ogm-cbf/ogm-cbf-pyramid/ros2_ws/src/ogm_cbf_kinematic_sim/ogm_cbf_kinematic_sim/map/pyramid/Frame_22_2.png"
VIS_SET = [
        ("cbf_traj_sigma_NA_pyr_1.txt",IMG, {'pyr_sigma' : -1, 'pyr_level' : 1}),    
        (None,IMG,{'vis_colorbar': True}),    
        #(None,IMG,{}),    
        (None,IMG,{'vis_legend': True, 'pyr_level' : 3}),    
        #
        ("cbf_traj_sigma_NA_pyr_2.txt",IMG, {'pyr_sigma' : -1, 'pyr_level' : 2}),    
        ("cbf_traj_sigma_8dot6_pyr_2.txt",IMG, {'pyr_sigma' : 8.6, 'pyr_level' : 2}),    
        ("cbf_traj_sigma_86_pyr_2.txt",IMG, {'pyr_sigma' : 86.0, 'pyr_level' : 2}),    
        #
        ("cbf_traj_sigma_NA_pyr_3.txt",IMG, {'pyr_sigma' : -1, 'pyr_level' : 3}),    
        ("cbf_traj_sigma_8dot6_pyr_3.txt",IMG, {'pyr_sigma' : 8.6, 'pyr_level' : 3}),    
        ("cbf_traj_sigma_86_pyr_3.txt",IMG, {'pyr_sigma' : 86.0, 'pyr_level' : 3}),    
        #
        #("cbf_traj_sigma_NA_pyr_5.txt",IMG, {'pyr_sigma' : -1, 'pyr_level' : 5}),    
        #("cbf_traj_sigma_8dot6_pyr_5.txt",IMG, {'pyr_sigma' : 8.6, 'pyr_level' : 5}),    
        #("cbf_traj_sigma_86_pyr_5.txt",IMG, {'pyr_sigma' : 86.0, 'pyr_level' : 5}),    
        #
        #("cbf_traj_sigma_NA_pyr_8.txt",IMG, {'pyr_sigma' : -1, 'pyr_level' : 8}),    
        #("cbf_traj_sigma_8dot6_pyr_8.txt",IMG, {'pyr_sigma' : 8.6, 'pyr_level' : 8}),    
        #("cbf_traj_sigma_86_pyr_8.txt",IMG, {'pyr_sigma' : 86.0, 'pyr_level' : 8}),    
        ]
vis_traj_set(VIS_SET)

