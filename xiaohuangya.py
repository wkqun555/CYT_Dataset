import pybullet as p
import matplotlib.pyplot as plt
import matplotlib.image as mp
import pybullet_data
import numpy as np
import random
import time
import math
import csv
import os
'''
import pybullet_object_models
from pybullet_object_models import ycb_objects
from pybullet_object_models import graspa_layouts
from pybullet_object_models import superquadric_objects
'''
dt = math.pi/4


allobjects=['a_colored_wood_blocks','a_cups','adjustable_wrench','a_lego_duplo','a_marbles','apple','a_toy_airplane',
                'banana','baseball','b_colored_wood_blocks','b_cups','bleach_cleanser','b_lego_duplo','bowl','b_toy_airplane',
                'c_cups','chips_can','c_lego_duplo','cracker_box','c_toy_airplane','d_cups','dice','d_lego_duplo','d_toy_airplane'
                      ,'e_cups','e_lego_duplo','e_toy_airplane','extra_large_clamp','f_cups','flat_screwdriver','f_lego_duplo',
                'foam_brick','fork','g_cups','gelatin_box','g_lego_duplo','golf_ball','hammer','h_cups','i_cups','j_cups',
               'knife','large_clamp','large_marker','lemon','master_chef_can','medium_clamp','mini_soccer_ball','mug','mustard_bottle',
                'nine_hole_peg_test','orange','padlock','peach','pear','phillips_screwdriver','pitcher_base','plate','plum',
                 'potted_meat_can','power_drill','pudding_box','racquetball','rubiks_cube','scissors','skillet_lid','softball','spatula',
                  'sponge','spoon','strawberry','sugar_box','tennis_ball','tomato_soup_can','tuna_fish_can','windex_bottle','wood_block']
p.connect(p.GUI)
for m in range(1,10):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(1. / 120.)
    # useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
    p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
    width = 512
    height = 512
    radius=5
    t = 0
    p.configureDebugVisualizer(shadowMapWorldSize=5)
    p.configureDebugVisualizer(shadowMapResolution=8192)
    p.loadURDF("tray/traybox.urdf")

    fov = 60
    aspect = width / height
    near = 0.02
    far = 0.8

    view_matrix = p.computeViewMatrix([0, 0, 0.7], [0, 0, 0], [1, 0, 0])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)


    index = random.sample(range(0,77),12)
    objname=[]
    objdata=[]
    for i in index:
        objname.append("YCB_%s" %(allobjects[i]))
        objdata.append(p.loadURDF(os.path.join("ycb_objects", allobjects[i], "model.urdf"), [random.uniform(-0.20,0.20),random.uniform(-0.20,0.20),random.uniform(0.05,0.5)]))
    time.sleep(10)
    p.setGravity(0, 0, -7)
    p.setRealTimeSimulation(1)

    time.sleep(4)

    for i in range(0,8):
        t = 0 + i*dt
        p.configureDebugVisualizer(lightPosition=[radius * math.sin(t), radius * math.cos(t), 3])
        p.stepSimulation()
        images = p.getCameraImage(width,
                                  height,
                                  viewMatrix=view_matrix,
                                  projectionMatrix=projection_matrix,
                                  lightDirection=[-0., -1., -1.],
                                  lightColor=[1., 1., 1.],
                                  lightDistance=2,
                                  shadow=1,
                                  renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
        depth_buffer_opengl = np.reshape(images[3], [width, height])
        depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
        seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
        time.sleep(5)

        mp.imsave('RGBImages/%s.png' % (m*8+i), rgb_opengl)
        mp.imsave('DepthImages/%s.png' % (m*8+i), depth_opengl)
        mp.imsave('SegImages/%s.png' % (m*8+i), seg_opengl)

        file_handle=open('BaseandOrien/'+'%s.csv' %(m*8+i),mode='w',encoding='utf-8',newline='')
        csv_writer = csv.writer(file_handle)
        for i in range(0,len(index)):
            baseandorien=p.getBasePositionAndOrientation(objdata[i])
            if abs(baseandorien[0][0])<0.28 and abs(baseandorien[0][1])<0.28 and baseandorien[0][2]>0:
                csv_writer.writerow([objname[i], str(baseandorien)])
        file_handle.close()

    #travel back
    #print('data generating'+' %s' %(m/125.)+'%')
    p.resetSimulation()
