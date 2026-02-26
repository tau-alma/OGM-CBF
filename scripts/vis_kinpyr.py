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

def get_background(img, pyr_level, pyr_sigma):

    h, w = img.shape

    x = cv2.distanceTransform(img, cv2.DIST_L2, 3, dstType=cv2.CV_32F)
    for k in range(1, pyr_level):
        
        if pyr_sigma > 0: 
            x = cv2.GaussianBlur(x, (0, 0), sigmaX=pyr_sigma, sigmaY=pyr_sigma)
        x = cv2.pyrDown(x)              # Gaussian+downsample on float32 meters

    if pyr_level == 1 and pyr_sigma > 0:
        x = cv2.GaussianBlur(x, (0, 0), sigmaX=pyr_sigma, sigmaY=pyr_sigma)

    x = cv2.resize(x, (h,w))
    
    x = (x / max(h,w) * 255).astype(np.uint8)
    x = cv2.applyColorMap(x, cv2.COLORMAP_JET)
    x[img < 127, :] = 0 

    return x    

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

    if 'clip' in kwargs:
        clip = kwargs['clip']
    else:
        clip = (0., 0., 0., 0.)

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
            cmap='plasma',
            #extent=(-.5+clip[0]*w, w-.5-clip[1]*w, h-.5-clip[2]*h,-.5+clip[3]*h)
            )
    #_patch = patches.Circle((260, 200), radius=20, transform=ax.transData)
    #_im.set_clip_path(_patch)

    idx = [0, 20, 40, 60, 80]

    kwargs = {
            "width" : 0.005,
    #        "headwidth" : 5.,
            }

    ax.plot(
            traj_xy[idx[0]:idx[-1],0],traj_xy[idx[0]:idx[-1],1],
            c='orange', label="Trajectory with Headings", linewidth=3)


    ax.quiver(
        traj_xy[idx,0], traj_xy[idx,1], traj_uv[idx,0], traj_uv[idx,1],
        angles="xy",
        color='orange',
        #label="Heading",
        **kwargs)

    clrs = ['red', 'green', 'blue']
    for l in range(ls):
        ax.quiver(
            sdf_pt[idx,0], sdf_pt[idx,1],
            sdf_grad_levels[idx,l*2], sdf_grad_levels[idx,l*2+1],
            #scale = scales[level],
            color=clrs[l],
            label=r"$\nabla\phi'$ Level $%d$" % (l+1),
            **kwargs)




    #leg = ax.legend(ncol=2, loc='upper center',  bbox_to_anchor=(0.4, 0.9))    
    #leg.get_frame().set_alpha(0)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.set_xticks([])
    ax.set_yticks([])
    
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

    fig = plt.figure(layout='tight', figsize=(10*w,10*h))
    subfigs = fig.subfigures(h,w,hspace=0.,wspace=0.)

    for ij, entry in enumerate(traj_set):
        i = ij // w
        j = ij % w
        traj_pth, img_pth, kwargs = entry
        vis_traj(subfigs[i, j], traj_pth, img_pth, **kwargs)

    plt.savefig("/tmp/kinset.pdf",
               bbox_inches='tight', 
               transparent=True,
               pad_inches=0)

IMG = "/data/repos/ogm-cbf/ogm-cbf-pyramid/ros2_ws/src/ogm_cbf_kinematic_sim/ogm_cbf_kinematic_sim/map/pyramid/Frame_22_2.png"
VIS_SET = [
        ("cbf_traj_sigma_NA_pyr_1.txt",IMG, {'pyr_sigma' : -1, 'pyr_level' : 1}),    
        ("cbf_traj_sigma_4dot3_pyr_1.txt",IMG, {'pyr_sigma' : 4.3, 'pyr_level' : 1}),    
        ("cbf_traj_sigma_8dot6_pyr_1.txt",IMG, {'pyr_sigma' : 8.6, 'pyr_level' : 1}),    
        #
        ("cbf_traj_sigma_NA_pyr_3.txt",IMG, {'pyr_sigma' : -1, 'pyr_level' : 3}),    
        ("cbf_traj_sigma_4dot3_pyr_1.txt",IMG, {'pyr_sigma' : 4.3, 'pyr_level' : 1}),    
        ("cbf_traj_sigma_8dot6_pyr_3.txt",IMG, {'pyr_sigma' : 8.6, 'pyr_level' : 3}),    
        ]
vis_traj_set(VIS_SET)

#vis_traj(
#        "/data/repos/ogm-cbf/ogm-cbf-mir/scripts/kinpyr_traj_s8.txt",
#        "/data/repos/ogm-cbf/ogm-cbf-pyramid/ros2_ws/src/ogm_cbf_kinematic_sim/ogm_cbf_kinematic_sim/map/pyramid/Frame_22_2.png",
#        )    
#vis_traj(
#        "/data/repos/ogm-cbf/ogm-cbf-mir/scripts/kinbase_traj_s8.txt",
#        "/data/repos/ogm-cbf/ogm-cbf-pyramid/ros2_ws/src/ogm_cbf_kinematic_sim/ogm_cbf_kinematic_sim/map/pyramid/Frame_22_2.png",
#        )    
#vis_traj(
#        "/tmp/cbf_traj.txt",
#        "/data/repos/ogm-cbf/ogm-cbf-pyramid/ros2_ws/src/ogm_cbf_kinematic_sim/ogm_cbf_kinematic_sim/map/pyramid/Frame_22_2.png",
#        )    
