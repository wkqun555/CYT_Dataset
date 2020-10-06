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
shit=['dice','YCB_adjustable_wrench','YCB_a_lego_duplo','YCB_c_lego_duplo','YCB_fork','YCB_knife','YCB_large_marker','YCB_padlock','YCB_spatula','YCB_sponge','YCB_spoon','YCB_tuna_fish_can']
realshit=['dice','large_marker','spoon']
p.connect(p.GUI)
'''
for i in legoshit:
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
    p.loadURDF(os.path.join(ycb_objects.getDataPath(), i, "model.urdf"), [0,0,0.3])

    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(1)

    time.sleep(10)

    #travel back
    print('data generating'+i)
    '''
for i in realshit:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(1. / 120.)
        # useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)
    p.loadURDF("plane100.urdf", useMaximalCoordinates=True)
    width = 512
    height = 512

    p.configureDebugVisualizer(shadowMapWorldSize=5)
    p.configureDebugVisualizer(shadowMapResolution=8192)
    p.loadURDF("tray/traybox.urdf")
    obj=p.loadURDF(os.path.join(os.path.join("ycb_objects"),i, "model.urdf"), [0.19,0.19,0.5])
    p.setGravity(0, 0, -8)
    p.setRealTimeSimulation(1)
    time.sleep(10)
    baseandorien = p.getBasePositionAndOrientation(obj)
    print(baseandorien)
    if baseandorien[0][0] < 0.28 and baseandorien[0][1] < 0.28 and baseandorien[0][2]>0:
        print(i+'is in the box!')
    else:
        print('something is wrong with'+i)

    p.resetSimulation()