import numpy as np
from scipy.spatial import distance
import random
from collections import Counter
from math import pi
from sklearn.preprocessing import MinMaxScaler
from RRT import *
import json
from RRT_UR10e import *
# finding Medians on the the x-y plane recursively  
def collect_medians_of_splits(data, max_depth, depth=1, medians=None):
    
    if medians is None:
        medians = []

    if depth > max_depth or len(data) <= 1:
        return medians

    x_values = [point[0] for point in data]
    median_x = np.median(x_values)
    medians.append(median_x)

    # [print(point[0], point[1], point[2]) for point in data]

    left_group = []
    right_group = []
    for point in data:
      if point[0] <= median_x:
        left_group.append([point[1], point[0]])
      if point[0] > median_x:
        right_group.append([point[1], point[0]])

    # left_group = [point for point in data if point[0] <= median_x]
    # right_group = [point for point in data if point[0] > median_x]

    collect_medians_of_splits(left_group, max_depth, depth + 1, medians)
    collect_medians_of_splits(right_group, max_depth, depth + 1, medians)

    return medians

#finding medians on the z-axis 
def collect_medians_z(data, max_depth, depth=1, medians_z=None):

    if medians_z is None:
        medians_z = []

    if depth > max_depth or len(data) <= 1:
        return medians_z

    z_values = [point[2] for point in data]
    median_z = np.median(z_values)
    medians_z.append(median_z)

    left_group = []
    right_group = []
    for point in data:
        if point[2] <= median_z:
            left_group.append(point)
        else:
            right_group.append(point)

    collect_medians_z(left_group, max_depth, depth + 1, medians_z)
    collect_medians_z(right_group, max_depth, depth + 1, medians_z)

    return medians_z

def all_medians(data,max_depth2D,max_depth3D):
    m1=collect_medians_of_splits(data,max_depth2D,depth=1,medians=None)
    m2=collect_medians_z(data,max_depth3D,depth=1,medians_z=None)
    return m1+m2

#finding the initial zone on the 2D plane
def detect_zone(point,medians,depth):
   if depth == 2:
        if point[0]>medians[0]:
                if point[1]>medians[2]:
                    Zg=3
                else:
                    Zg=2
        else:
                if point[1]>medians[1]:
                    Zg=1
                else:
                    Zg=0
        return Zg
   
   if depth == 3:
        if point[0]>medians[0]:
            if point[1]> medians[4]:
                if point[0]>medians[6]:
                    Zg=7
                else:
                    Zg=5
            else:
                if point[0]>medians[5]:
                    Zg=6
                else:
                    Zg=4
        else:
            if point[1]>medians[1]:
                if point[0]>medians[3]:
                    Zg=3
                else:
                    Zg=1
            else:
                if point[0]>medians[2]:
                    Zg=2
                else:
                    Zg=0
        return(Zg)
   
   if depth == 4:
        Zg=-1
        #Right
        if point[0]>medians[0]:
            if point[1]> medians[8]:
                if point[0]>medians[12]:
                    if point[1]>medians[14]:
                        Zg=15
                    else:
                        Zg=14
                else:
                    if point[1]>medians[13]:
                        Zg=11
                    else:
                        Zg=10
            if point[1]< medians[8]:
                if point[0]>medians[9]:
                    if point[1]>medians[11]:
                        Zg=13
                    else:
                        Zg=12
                else:
                    if point[1]>medians[10]:
                        Zg=9
                    else:
                        Zg=8
        #Left        
        if point[0]<medians[0]:
            if point[1]> medians[1]:
                if point[0]>medians[5]:
                    if point[1]>medians[7]:
                        Zg=7
                    else:
                        Zg=6
                else:
                    if point[1]>medians[6]:
                        Zg=3
                    else:
                        Zg=2
            if point[1]< medians[1]:
                if point[0]>medians[2]:
                    if point[1]>medians[4]:
                        Zg=5
                    else:
                        Zg=4
                else:
                    if point[1]>medians[3]:
                        Zg=1
                    else:
                        Zg=0 
        return(Zg)
   
#setting the final zone based on the z-axis
def Final_zone(point,medians_XY,mediansZ,depth2D,depthZ):
    Z2d=detect_zone(point,medians_XY,depth2D)
    if depthZ == 1:
        if point[2]>mediansZ[0]:
            Z2d += depth2D**2
    
    if depthZ ==2:
        if point[2]<mediansZ[0]:
            if point[2]>mediansZ[1]:
                Z2d += 2**depth2D
        if point[2]>mediansZ[0]:
            if point[2]>mediansZ[2]:
                Z2d += 3*(2**depth2D)
            if point[2]<mediansZ[2]:
                Z2d += 2*(2**depth2D)

    return Z2d

def create_zone(data,max_depth2D,max_depth3D,boundry):
    medians= all_medians(data,max_depth2D,max_depth3D)
    depth= max_depth2D*max_depth3D

    if max_depth2D == 2:
        #medians= [x, yL, yR, z]
        Z0=[0,0,0,medians[0],medians[1],medians[2]] #[x0,y0,z0,x1,y1,z1]
        Z1=[0,medians[1],0,medians[0],boundry,medians[3]]
        Z2=[medians[0],0,0,boundry,medians[2],medians[3]]
        Z3=[medians[0],medians[2],0,boundry,boundry,medians[3]]
        #-
        Z4=[0,0,medians[3],medians[0],medians[1],boundry] #[x0,y0,z0,x1,y1,z1]
        Z5=[0,medians[1],medians[3],medians[0],boundry,boundry]
        Z6=[medians[0],0,medians[3],boundry,medians[2],boundry]
        Z7=[medians[0],medians[2],medians[3],boundry,boundry,boundry]

        zones = [Z0,Z1,Z2,Z3,Z4,Z5,Z6,Z7]
        # for i in range(8):  # 2**3 is 8
        #     zones.append(eval(f'Z{i}'))
        return zones
    if max_depth2D ==3:
        #- [---,z1=medians[7],z2=medians[8],z3=medians[9]]
        # 0 --> medians [8]
        Z0=[0,0,0,medians[2],medians[1],medians[8]]
        Z1=[0,medians[1],0,medians[3],boundry,medians[8]]
        Z2=[medians[2],0,0,medians[0],medians[1],medians[8]]
        Z3=[medians[3],medians[1],0,medians[0],boundry,medians[8]]
        Z4=[medians[0],0,0,medians[5],medians[4],medians[8]]
        Z5=[medians[0],medians[4],0,medians[6],boundry,medians[8]]
        Z6=[medians[5],0,0,boundry,medians[4],medians[8]]
        Z7=[medians[6],medians[4],0,boundry,boundry,medians[8]]
        # medians[8]--> medians[7]
        Z8=[0,0,medians[8],medians[2],medians[1],medians[7]]
        Z9=[0,medians[1],medians[8],medians[3],boundry,medians[7]]
        Z10=[medians[2],0,medians[8],medians[0],medians[1],medians[7]]
        Z11=[medians[3],medians[1],medians[8],medians[0],boundry,medians[7]]
        Z12=[medians[0],0,medians[8],medians[5],medians[4],medians[7]]
        Z13=[medians[0],medians[4],medians[8],medians[6],boundry,medians[7]]
        Z14=[medians[5],0,medians[8],boundry,medians[4],medians[7]]
        Z15=[medians[6],medians[4],medians[8],boundry,boundry,medians[7]]
        # medians [7]--> medians[9]
        Z16=[0,0,medians[7],medians[2],medians[1],medians[9]]
        Z17=[0,medians[1],medians[7],medians[3],boundry,medians[9]]
        Z18=[medians[2],0,medians[7],medians[0],medians[1],medians[9]]
        Z19=[medians[3],medians[1],medians[7],medians[0],boundry,medians[9]]
        Z20=[medians[0],0,medians[7],medians[5],medians[4],medians[9]]
        Z21=[medians[0],medians[4],medians[7],medians[6],boundry,medians[9]]
        Z22=[medians[5],0,medians[7],boundry,medians[4],medians[9]]
        Z23=[medians[6],medians[4],medians[7],boundry,boundry,medians[9]]
        #medians [9] --> boundry
        Z24=[0,0,medians[9],medians[2],medians[1],boundry]
        Z25=[0,medians[1],medians[9],medians[3],boundry,boundry]
        Z26=[medians[2],0,medians[9],medians[0],medians[1],boundry]
        Z27=[medians[3],medians[1],medians[9],medians[0],boundry,boundry]
        Z28=[medians[0],0,medians[9],medians[5],medians[4],boundry]
        Z29=[medians[0],medians[4],medians[9],medians[6],boundry,boundry]
        Z30=[medians[5],0,medians[9],boundry,medians[4],boundry]
        Z31=[medians[6],medians[4],medians[9],boundry,boundry,boundry]
        zones = []
        # for i in range(32):  # 2**5 is 8
        #     zones.append(eval(f'Z{i}'))
        zones= [Z0,
                Z1,
                Z2,
                Z3,
                Z4,
                Z5,
                Z6,
                Z7,
                Z8,
                Z9,
                Z10,
                Z11,
                Z12,
                Z13,
                Z14,
                Z15,
                Z16,
                Z17,
                Z18,
                Z19,
                Z20,
                Z21,
                Z22,
                Z23,
                Z24,
                Z25,
                Z26,
                Z27,
                Z28,
                Z29,
                Z30,
                Z31]   
        return zones 
    if max_depth2D ==4:

        # 0 -->medians[16]
        Z0=[0,0,0,medians[2],medians[3],medians[16]]
        Z1=[0,medians[3],0,medians[2],medians[1],medians[16]]
        Z2=[0,medians[1],0,medians[5],medians[6],medians[16]]
        Z3=[0,medians[6],0,medians[5],boundry,medians[16]]
        Z4=[medians[2],0,0,medians[0],medians[4],medians[16]]
        Z5=[medians[2],0,0,medians[0],medians[1],medians[16]]
        Z6=[medians[5],medians[1],0,medians[0],medians[7],medians[16]]
        Z7=[medians[5],medians[7],0,medians[0],boundry,medians[16]]
        Z8=[medians[0],0,0,medians[9],medians[10],medians[16]]
        Z9=[medians[0],medians[10],0,medians[9],medians[8],medians[16]]
        Z10=[medians[0],medians[8],0,medians[12],medians[13],medians[16]]
        Z11=[medians[0],medians[13],0,medians[12],boundry,medians[16]]
        Z12=[medians[9],0,0,boundry,medians[11],medians[16]]
        Z13=[medians[9],medians[11],0,boundry,medians[8],medians[16]]
        Z14=[medians[12],medians[8],0,boundry,medians[14],medians[16]]
        Z15=[medians[12],medians[14],0,boundry,boundry,medians[16]]
        # medians[16] --> medians[15]
        Z16=[0,0,medians[16],medians[2],medians[3],medians[15]]
        Z17=[0,medians[3],medians[16],0,medians[2],medians[1],medians[15]]
        Z18=[0,medians[1],medians[16],medians[5],medians[6],medians[15]]
        Z19=[0,medians[6],medians[16],medians[5],boundry,medians[15]]
        Z20=[medians[2],0,medians[16],medians[0],medians[4],medians[15]]
        Z21=[medians[2],0,medians[16],medians[0],medians[1],medians[15]]
        Z22=[medians[5],medians[1],medians[16],medians[0],medians[7],medians[15]]
        Z23=[medians[5],medians[7],medians[16],medians[0],boundry,medians[15]]
        Z24=[medians[0],0,medians[16],medians[9],medians[10],medians[15]]
        Z25=[medians[0],medians[10],medians[16],medians[9],medians[8],medians[15]]
        Z26=[medians[0],medians[8],medians[16],medians[12],medians[13],medians[15]]
        Z27=[medians[0],medians[13],medians[16],medians[12],boundry,medians[15]]
        Z28=[medians[9],0,medians[16],boundry,medians[11],medians[15]]
        Z29=[medians[9],medians[11],medians[16],boundry,medians[8],medians[15]]
        Z30=[medians[12],medians[8],medians[16],boundry,medians[14],medians[15]]
        Z31=[medians[12],medians[14],medians[16],boundry,boundry,medians[15]]
        # medians [15] --> medians[17]
        Z32=[0,0,medians[15],medians[2],medians[3],medians[17]]
        Z33=[0,medians[3],medians[15],0,medians[2],medians[1],medians[17]]
        Z34=[0,medians[1],medians[15],medians[5],medians[6],medians[17]]
        Z35=[0,medians[6],medians[15],medians[5],boundry,medians[17]]
        Z36=[medians[2],0,medians[15],medians[0],medians[4],medians[17]]
        Z37=[medians[2],0,medians[15],medians[0],medians[1],medians[17]]
        Z38=[medians[5],medians[1],medians[15],medians[0],medians[7],medians[17]]
        Z39=[medians[5],medians[7],medians[15],medians[0],boundry,medians[17]]
        Z40=[medians[0],0,medians[15],medians[9],medians[10],medians[17]]
        Z41=[medians[0],medians[10],medians[15],medians[9],medians[8],medians[17]]
        Z42=[medians[0],medians[8],medians[15],medians[12],medians[13],medians[17]]
        Z43=[medians[0],medians[13],medians[15],medians[12],boundry,medians[17]]
        Z44=[medians[9],0,medians[15],boundry,medians[11],medians[17]]
        Z45=[medians[9],medians[11],medians[15],boundry,medians[8],medians[17]]
        Z46=[medians[12],medians[8],medians[15],boundry,medians[14],medians[17]]
        Z47=[medians[12],medians[14],medians[15],boundry,boundry,medians[17]]        
        # medians [17] --> boundry
        Z48=[0,0,medians[17],medians[2],medians[3],boundry]
        Z49=[0,medians[3],medians[17],0,medians[2],medians[1],boundry]
        Z50=[0,medians[1],medians[17],medians[5],medians[6],boundry]
        Z51=[0,medians[6],medians[17],medians[5],boundry,boundry]
        Z52=[medians[2],0,medians[17],medians[0],medians[4],boundry]
        Z53=[medians[2],0,medians[17],medians[0],medians[1],boundry]
        Z54=[medians[5],medians[1],medians[17],medians[0],medians[7],boundry]
        Z55=[medians[5],medians[7],medians[17],medians[0],boundry,boundry]
        Z56=[medians[0],0,medians[17],medians[9],medians[10],boundry]
        Z57=[medians[0],medians[10],medians[17],medians[9],medians[8],boundry]
        Z58=[medians[0],medians[8],medians[17],medians[12],medians[13],boundry]
        Z59=[medians[0],medians[13],medians[17],medians[12],boundry,boundry]
        Z60=[medians[9],0,medians[17],boundry,medians[11],boundry]
        Z61=[medians[9],medians[11],medians[17],boundry,medians[8],boundry]
        Z62=[medians[12],medians[8],medians[17],boundry,medians[14],boundry]
        Z63=[medians[12],medians[14],medians[17],boundry,boundry,boundry]
        zones = []
        # for i in range(64):  # 2**5 is 8
        #     zones.append(eval(f'Z{i}'))
        zones= [Z0,
                Z1,
                Z2,
                Z3,
                Z4,
                Z5,
                Z6,
                Z7,
                Z8,
                Z9,
                Z10,
                Z11,
                Z12,
                Z13,
                Z14,
                Z15,
                Z16,
                Z17,
                Z18,
                Z19,
                Z20,
                Z21,
                Z22,
                Z23,
                Z24,
                Z25,
                Z26,
                Z27,
                Z28,
                Z29,
                Z30,
                Z31,
                Z32,
                Z33,
                Z34,
                Z35,
                Z36,
                Z37,
                Z38,
                Z39,
                Z40,
                Z41,
                Z42,
                Z43,
                Z44,
                Z45,
                Z46,
                Z47,
                Z48,
                Z49,
                Z50,
                Z51,
                Z52,
                Z53,
                Z54,
                Z55,
                Z56,
                Z57,
                Z58,
                Z59,
                Z60,
                Z61,
                Z62,
                Z63]
        return zones

def get_adjacency_matrix(zones,depth2D):

    if depth2D == 2:
        Z0, Z1, Z2, Z3,Z4,Z5,Z6,Z7 = zones
        adjacency_matrix = np.zeros((8,8))
        for i in range(0,8,4):
            adjacency_matrix[0+i,1+i],adjacency_matrix[1+i,0+i]=1,1
            adjacency_matrix[2+i,3+i],adjacency_matrix[3+i,2+i]=1,1
            adjacency_matrix[0+i,2+i],adjacency_matrix[2+i,0+i]=1,1
            adjacency_matrix[1+i,3+i],adjacency_matrix[3+i,1+i]=1,1
            if Z1[1]>Z3[1]:
                adjacency_matrix[0+i,3+i]=1
                adjacency_matrix[3+i,0+i]=1
            else:
                adjacency_matrix[1+i,2+i]=1
                adjacency_matrix[2+i,1+i]=1

        adjacency_matrix[0,4],adjacency_matrix[4,0]=1,1
        adjacency_matrix[1,5],adjacency_matrix[5,1]=1,1    
        adjacency_matrix[2,6],adjacency_matrix[6,2]=1,1
        adjacency_matrix[3,7],adjacency_matrix[7,3]=1,1

        return adjacency_matrix

    if depth2D == 3:
        
        Z0,Z1,Z2,Z3,Z4,Z5,Z6,Z7,Z8,Z9,Z10,Z11,Z12,Z13,Z14,Z15,Z16,Z17,Z18,Z19,Z20,Z21,Z22,Z23,Z24,Z25,Z26,Z27,Z28,Z29,Z30,Z31=zones

        adjacency_matrix = np.zeros((32,32))
        for j in range(0,31,8):
            for i in range(6):
                adjacency_matrix[i+j,i+j+2]=1
                adjacency_matrix[i+2+j,i+j]=1
            for i in range(0,7,2):
                adjacency_matrix[i+j,i+1+j]=1
                adjacency_matrix[i+1+j,i+j]=1
            
            if Z3[0] < Z0[3]: #[x0,y0,x1,y1] -->[x0,y0,z0,x1,y1,z1]

                adjacency_matrix[0+j,3+j]=1
                adjacency_matrix[3+j,0+j]=1
            else:
                adjacency_matrix[1+j,2+j]=1
                adjacency_matrix[2+j,1+j]=1
            
            if Z7[0] < Z4[3]:
                adjacency_matrix[4+j,7+j]=1
                adjacency_matrix[7+j,4+j]=1
            else:
                adjacency_matrix[5+j,6+j]=1
                adjacency_matrix[6+j,5+j]=1
    
            if Z3[1] < Z4[4]:

                adjacency_matrix[3+j,4+j]=1
                adjacency_matrix[4+j,3+j]=1
            else:
                adjacency_matrix[2+j,5+j]=1
                adjacency_matrix[5+j,2+j]=1
            
        for j in range(0,23,8):
            
            adjacency_matrix[j,j+8],adjacency_matrix[j+8,j]=1,1
        
        return adjacency_matrix
    
    if depth2D ==4:
        Z0,Z1,Z2, Z3,Z4,Z5,Z6,Z7, Z8,Z9, Z10,Z11, Z12, Z13, Z14, Z15, Z16, Z17,Z18,Z19,Z20,Z21,Z22,Z23,Z24,Z25,Z26,Z27,Z28,Z29,Z30,Z31,Z32,Z33,Z34,Z35,Z36,Z37,Z38,Z39,Z40,Z41,Z42,Z43,Z44,Z45,Z46,Z47, Z48,Z49,Z50,Z51,Z52,Z53,Z54, Z55,Z56,Z57,Z58,Z59,Z60,Z61,Z62,Z63 = zones
        # for i, zone in enumerate(zones):
        #     globals()[f'Z{i}'] = zone

        adjacency_matrix = np.zeros((64,64))

        for j in range(0,63,16):
            for i in range(12):
                adjacency_matrix[i+j,i+4+j]=1
                adjacency_matrix[i+4+j,i+j]=1
            for i in range(0,16,4):
                adjacency_matrix[i+j,i+1+j]=1
                adjacency_matrix[i+1+j,i+j]=1

                adjacency_matrix[i+1+j,i+2+j]=1
                adjacency_matrix[i+2+j,i+1+j]=1

                adjacency_matrix[i+2+j,i+3+j]=1
                adjacency_matrix[i+3+j,i+2+j]=1
        #############  Left  ################
            if Z5[1] < Z1[1]:
                adjacency_matrix[0+j,5+j]=1
                adjacency_matrix[5+j,0+j]=1
            else:
                adjacency_matrix[1+j,4+j]=1
                adjacency_matrix[4+j,1+j]=1
            if Z7[1] < Z3[1]:
                adjacency_matrix[2+j,7+j]=1
                adjacency_matrix[7+j,2+j]=1
            else:
                adjacency_matrix[3+j,6+j]=1
                adjacency_matrix[6+j,3+j]=1
            if Z6[0] < Z5[0]:
                adjacency_matrix[1+j,6+j]=1
                adjacency_matrix[6+j,1+j]=1
            else:
                adjacency_matrix[2+j,5+j]=1
                adjacency_matrix[5+j,2+j]=1
        ############ Right ###################
            if Z13[1] < Z9[1]:
                adjacency_matrix[8+j,13+j]=1
                adjacency_matrix[13+j,8+j]=1
            else:
                adjacency_matrix[9+j,12+j]=1
                adjacency_matrix[12+j,9+j]=1
            if Z15[1] < Z11[1]:
                adjacency_matrix[10+j,15+j]=1
                adjacency_matrix[15+j,10+j]=1
            else:
                adjacency_matrix[11+j,14+j]=1
                adjacency_matrix[14+j,11+j]=1
            if Z14[0] < Z13[0]:
                adjacency_matrix[9+j,14+j]=1
                adjacency_matrix[14+j,9+j]=1
            else:
                adjacency_matrix[10+j,13+j]=1
                adjacency_matrix[13+j,10+j]=1
        ############  L|R  ###################
            if Z4[4] < Z9[1]:
                adjacency_matrix[8+j,5+j]=1
                adjacency_matrix[5+j,8+j]=1
            else:
                adjacency_matrix[9+j,4+j]=1
                adjacency_matrix[4+j,9+j]=1
            if Z5[4] < Z10[1]:
                adjacency_matrix[6+j,9+j]=1
                adjacency_matrix[9+j,6+j]=1
            else:
                adjacency_matrix[5+j,10+j]=1
                adjacency_matrix[10+j,5+j]=1
            if Z6[4] < Z11[1]:
                adjacency_matrix[7+j,10+j]=1
                adjacency_matrix[10+j,7+j]=1
            else:
                adjacency_matrix[6+j,11+j]=1
                adjacency_matrix[11+j,6+j]=1

        for j in range(0,47,16):
            adjacency_matrix[j,j+16],adjacency_matrix[j+16,j]=1,1
        
        return adjacency_matrix 

def distances(zones,goal,normalized=True):
    distances = []
    for zone in zones:
        center_x = (zone[0] + zone[3]) / 2
        center_y = (zone[1] + zone[4]) / 2
        center_z = (zone[2] + zone[5]) / 2
        dist= distance.euclidean(goal, [center_x, center_y,center_z])
        distances.append(dist)
    
    if normalized:
        distances = [distan/max(distances) for distan in distances]
    
    return distances

def get_DofZ(data,medians_XY,mediansZ,depth2D,depthZ,zones):
    
    if depth2D == 2:   
        Z0, Z1, Z2, Z3,Z4,Z5,Z6,Z7 = zones
        L=[]
        for i in range(len(data)):
            L+=[Final_zone(data[i], medians_XY,mediansZ,depth2D,depthZ)]
        count = Counter(L)
        DZ = []
        for i in range(len(zones)):
            Z_i = eval(f'Z{i}')  # Dynamically access Z_i
            denominator = (Z_i[3] - Z_i[0]) * (Z_i[4] - Z_i[1]) * (Z_i[5] - Z_i[2])
            DZ_i =  10 *count[i]* pi / denominator
            DZ.append(DZ_i)
        
        scaler = MinMaxScaler()
        DZ_normalized = scaler.fit_transform(np.array(DZ).reshape(-1, 1)).flatten()
        return DZ_normalized 

    if depth2D == 3:   
        Z0,Z1,Z2,Z3,Z4,Z5,Z6,Z7,Z8,Z9,Z10,Z11,Z12,Z13,Z14,Z15,Z16,Z17,Z18,Z19,Z20,Z21,Z22,Z23,Z24,Z25,Z26,Z27,Z28,Z29,Z30,Z31=zones

        L=[]
        for i in range(len(data)):
            L+=[Final_zone(data[i], medians_XY,mediansZ,depth2D,depthZ)]
        count = Counter(L)
        DZ = []
        for i in range(len(zones)):
            Z_i = eval(f'Z{i}')  # Dynamically access Z_i
            denominator = (Z_i[3] - Z_i[0]) * (Z_i[4] - Z_i[1]) * (Z_i[5] - Z_i[2])
            DZ_i =  10 *count[i]* pi / denominator
            DZ.append(DZ_i)
        
        scaler = MinMaxScaler()
        DZ_normalized = scaler.fit_transform(np.array(DZ).reshape(-1, 1)).flatten()
        return DZ_normalized 

    if depth2D == 4:
        Z0,Z1,Z2, Z3,Z4,Z5,Z6,Z7, Z8,Z9, Z10,Z11, Z12, Z13, Z14, Z15, Z16, Z17,Z18,Z19,Z20,Z21,Z22,Z23,Z24,Z25,Z26,Z27,Z28,Z29,Z30,Z31,Z32,Z33,Z34,Z35,Z36,Z37,Z38,Z39,Z40,Z41,Z42,Z43,Z44,Z45,Z46,Z47, Z48,Z49,Z50,Z51,Z52,Z53,Z54, Z55,Z56,Z57,Z58,Z59,Z60,Z61,Z62,Z63 = zones
        L=[]
        for i in range(len(data)):
            L+=[Final_zone(data[i], medians_XY,mediansZ,depth2D,depthZ)]
        count = Counter(L)
        if count == 0:
            print("zero points")
            return 0
        DZ = []
        for i in range(len(zones)):
            Z_i = eval(f'Z{i}')  # Dynamically access Z_i
            denominator = (Z_i[3] - Z_i[0]) * (Z_i[4] - Z_i[1]) * (Z_i[5] - Z_i[2])
            if denominator == 0 or np.isclose(denominator, 0):
                DZ_i = 0
            else:
                DZ_i = 10 * count[i] * pi / denominator
            DZ.append(DZ_i)
        
        scaler = MinMaxScaler()
        DZ_normalized = scaler.fit_transform(np.array(DZ).reshape(-1, 1)).flatten()
        return DZ_normalized

def setReward(adjacency_matrix, DZ_normalized,dist,depth):
    
    if depth == 2:
        allStateActions = np.full((8, 8), -1000,dtype=float)
        #Obtain allstateActions
        for i in range(len(allStateActions)):
            for j in range(len(allStateActions)):
                if adjacency_matrix[i,j]==1:
                    allStateActions[i,j]= -10* DZ_normalized[j] -0.5*dist[j]

        return allStateActions
    
    if depth == 3:
        allStateActions = np.full((32, 32), -1000,dtype=float)
        #Obtain allstateActions
        for i in range(len(allStateActions)):
            for j in range(len(allStateActions)):
                if adjacency_matrix[i,j]==1:
                    allStateActions[i,j]= -1e5* DZ_normalized[j] -1e5*dist[j]
        return allStateActions

    if depth == 4:
        allStateActions = np.full((64, 64), -1000,dtype=float)
        #Obtain allstateActions
        for i in range(len(allStateActions)):
            for j in range(len(allStateActions)):
                if adjacency_matrix[i,j]==1:
                    allStateActions[i,j]= -10* DZ_normalized[j] -0.5*dist[j]
        return allStateActions
    
def environment(allstateActions,state,newstate, goal_zone):
    if allstateActions[state,newstate] != -1000:
        if newstate == goal_zone:
            reward= +100
        else:
            reward= allstateActions[state,newstate]
    else:
            reward= -1000
    return newstate,reward

def choose_random_index_non_zero(matrix):
    if all(element9==0 for element9 in matrix):
        return 0
    else:
        non_zero_indices = np.where(matrix != 0)[0]
        random_index = np.random.choice(non_zero_indices)
        return random_index

def policyImprovementUsing(Q):
    numStates = int(len(Q))
    policy = np.zeros(numStates)
    for s in range(0,numStates):
        QQ = Q[s]
        mx = max(QQ)
        idx = [i if (QQ[i] == mx) else 0 for i in range(len(QQ))]
        idx_arr = np.array(idx)
        policy[s] = choose_random_index_non_zero(idx_arr)
    return policy

def train(max_depth, episodes, alpha, gamma, epsilon, allStateActions, goal):
    if max_depth == 2:
        num_states = 8
    elif max_depth == 3:
        num_states = 32
    elif max_depth == 4:
        num_states = 64
    print("train---------")    
    Q = np.zeros((num_states, num_states))
    policy = np.zeros(num_states)
    for i in range(num_states):
        policy[i] = random.randint(0, num_states-1)
    for episode in range(episodes):
        print(episode)
        state = random.randint(0, num_states-1)
        while True:

            rnd = random.random()
            if rnd > epsilon:
                A = int(policy[state])
            else:
                A = int(random.randint(0, num_states-1))
            newState, reward = environment(allStateActions, state, A, goal)
            Q[state, newState] = Q[state, newState]+ alpha*(reward + gamma*max(Q[newState]) - Q[state, newState])
            state = newState
            policy = policyImprovementUsing(Q)
            if(newState==goal):
                break
    return Q, policy

def get_final_policy(depth,Q,policy):
    if depth == 2:
        numStates = 8
    elif depth == 3:
        numStates = 32
    elif depth == 4:
        numStates = 64
    for s in range(0, numStates):
        QQ = Q[s]
        mx = max(QQ)
        idx = [i if (QQ[i] == mx) else 0 for i in range(len(QQ))]
        idx_arr = np.array(idx)
        policy[s] = choose_random_index_non_zero(idx_arr)
    return policy

def ZoneConnectivity(zones, obstacle, adjacency_matrix, gammaC):
    num_zones = len(zones)
    for i in range(num_zones):
        for j in range(num_zones):
            if adjacency_matrix[i, j] == 1:
                print(f"zone {i} and zone {j}")
                if not is_accessible(zones[i], zones[j], obstacle, gammaC):
                    print("they are not accessible")
                    adjacency_matrix[i, j] = 0
                    adjacency_matrix[j, i] = 0
    return adjacency_matrix

def is_accessible(zone_i, zone_j, obstacles, gamma_C, delta=1):
    """
    Determines if two zones are accessible to each other in 3D.
    """
    
    # Determine the common border and get the obstacle points along it
    common_border = get_common_border_points_3D(zone_i, zone_j, obstacles, delta)
    
    print(len(common_border))
    if len(common_border) < 3:
        return True
    
    # Check maximum gap between obstacles on the common border
    max_gap = 0
    common_border.sort(key=lambda p: (p[0], p[1], p[2]))  # Sort by coordinates
    for k in range(len(common_border) - 1):
        gap = np.linalg.norm(np.array(common_border[k]) - np.array(common_border[k + 1]))
        if gap > max_gap:
            max_gap = gap
    return max_gap >= gamma_C

def get_common_border_points_3D(zone_i, zone_j, obstacles, delta):
    """
    Gets the obstacle points along the common border of two zones in 3D.
    """
    common_border_points = []
    for x, y, z in obstacles:
        if is_within_vertical_border_3D(zone_i, zone_j, x, y, z, delta) or \
           is_within_horizontal_border_3D(zone_i, zone_j, x, y, z, delta) or \
           is_within_depth_border_3D(zone_i, zone_j, x, y, z, delta):
            common_border_points.append((x, y, z))
    return common_border_points

def has_common_border_3D(zone_i, zone_j):
    """
    Checks if two zones have a common border in 3D.
    """
    return (
        (zone_i[3] == zone_j[0] or zone_i[0] == zone_j[3]) or
        (zone_i[4] == zone_j[1] or zone_i[1] == zone_j[4]) or
        (zone_i[5] == zone_j[2] or zone_i[2] == zone_j[5])
    )

def is_within_vertical_border_3D(zone_i, zone_j, x, y, z, delta):
    """
    Checks if a point is within the vertical common border of two zones in 3D.
    """
    common_x_border = zone_i[3] if zone_i[3] == zone_j[0] else zone_i[0]
    return (
        common_x_border - delta <= x <= common_x_border + delta and
        min(zone_i[1], zone_j[1]) <= y <= max(zone_i[4], zone_j[4]) and
        min(zone_i[2], zone_j[2]) <= z <= max(zone_i[5], zone_j[5])
    )

def is_within_horizontal_border_3D(zone_i, zone_j, x, y, z, delta):
    """
    Checks if a point is within the horizontal common border of two zones in 3D.
    """
    common_y_border = zone_i[4] if zone_i[4] == zone_j[1] else zone_i[1]
    return (
        common_y_border - delta <= y <= common_y_border + delta and
        min(zone_i[0], zone_j[0]) <= x <= max(zone_i[3], zone_j[3]) and
        min(zone_i[2], zone_j[2]) <= z <= max(zone_i[5], zone_j[5])
    )

def is_within_depth_border_3D(zone_i, zone_j, x, y, z, delta):
    """
    Checks if a point is within the depth common border of two zones in 3D.
    """
    common_z_border = zone_i[5] if zone_i[5] == zone_j[2] else zone_i[2]
    return (
        common_z_border - delta <= z <= common_z_border + delta and
        min(zone_i[0], zone_j[0]) <= x <= max(zone_i[3], zone_j[3]) and
        min(zone_i[1], zone_j[1]) <= y <= max(zone_i[4], zone_j[4])
    )

# def is_point_too_close_to_obstacles(point, obstacle_list, min_distance=5):
#     radius= 1 #radius of obstacles
#     for obstacle in obstacle_list:
#         ox, oy, oz = obstacle
#         distance_po = np.sqrt((point[0] - ox)**2 + (point[1] - oy)**2 + (point[2] - oz)**2)
#         if distance_po < (radius + min_distance):
#             return True
#     return False
def is_point_too_close_to_obstacles(point, obstacle_list, min_distance):
    
    for obstacle in obstacle_list:
        if obstacle.point_in_rect(point, min_distance):
            return True
    return False

def generate_safe_sub_goals_box(Z_next, obstacle_list, goal, m, min_distance=5,greedy=True):
    safe_sub_goals = []
    while len(safe_sub_goals) < m:
        candidate_point = (
            random.uniform(Z_next[0], Z_next[3]), 
            random.uniform(Z_next[1], Z_next[4]), 
            random.uniform(Z_next[2], Z_next[5])
        )
        if not is_point_too_close_to_obstacles(candidate_point, obstacle_list, min_distance):
            safe_sub_goals.append(candidate_point)
    #choose one of them
    if greedy:
        # Choose the subgoal nearest to the goal
        safe_sub_goals.sort(key=lambda point: np.linalg.norm(np.array(point) - np.array(goal)))
        return safe_sub_goals[0]  
    
    return random.choice(safe_sub_goals)

def generate_safe_sub_goals(Z_next, obstacle_list, goal, m, min_distance=5,greedy=True):
    safe_sub_goals = []
    while len(safe_sub_goals) < m:
        candidate_point = (
            random.uniform(Z_next[0], Z_next[3]), 
            random.uniform(Z_next[1], Z_next[4]), 
            random.uniform(Z_next[2], Z_next[5])
        )
        if not is_point_too_close_to_obstacles(candidate_point, obstacle_list, min_distance):
            safe_sub_goals.append(candidate_point)
    #choose one of them
    if greedy:
        # Choose the subgoal nearest to the goal
        safe_sub_goals.sort(key=lambda point: np.linalg.norm(np.array(point) - np.array(goal)))
        return safe_sub_goals[0]  
    
    return random.choice(safe_sub_goals)

def assign_obstacles_to_zone(data, xmin, ymin, zmin, xmax, ymax, zmax):
    obstacles_in_zone = []

    for obstacle in data:
        x, y, z = obstacle
        if xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax:
            obstacles_in_zone.append(obstacle)
    
    return obstacles_in_zone

def simulate3D(zones,policy,data,start,startZone,goal,goalZone):
    nexZone=int(policy[startZone])
    goalZone=int(goalZone)
    nexZone=int(policy[startZone])
    T=0
    path=[]
    while nexZone != goalZone:
        #obstacles=assign_obstacles_to_zone(data,min(zones[startZone][0],zones[nexZone][0]),min(zones[startZone][1],zones[nexZone][1]),min(zones[startZone][2],zones[nexZone][2]),max(zones[startZone][0],zones[nexZone][0]),max(zones[startZone][1],zones[nexZone][1]),max(zones[startZone][2],zones[nexZone][2]))
        obstacles = assign_obstacles_to_zone(
            data,
            float(min(zones[int(startZone)][0], zones[int(nexZone)][0])),
            float(min(zones[int(startZone)][1], zones[int(nexZone)][1])),
            float(min(zones[int(startZone)][2], zones[int(nexZone)][2])),
            float(max(zones[int(startZone)][3], zones[int(nexZone)][3])),
            float(max(zones[int(startZone)][4], zones[int(nexZone)][4])),
            float(max(zones[int(startZone)][5], zones[int(nexZone)][5]))
        )
        SubGoal= generate_safe_sub_goals(zones[int(nexZone)],obstacles,goal,m=10,min_distance=5,greedy=True)
        print("next Subgoal",SubGoal)
        episodetime,newPath, done,iteration_count= RRT3D(start,SubGoal,zones[int(startZone)],zones[int(nexZone)],obstacles)  
        if not done:
            return T,path, 0,iteration_count
        if newPath is not None:
            path += [newPath]
        
        startZone=nexZone
        nexZone=int(policy[startZone])
        start= SubGoal
        T += episodetime
        path += newPath

    if nexZone == goalZone:
        #obstacles=assign_obstacles_to_zone(data,min(zones[startZone][0],zones[nexZone][0]),min(zones[startZone][1],zones[nexZone][1]),max(zones[startZone][0],zones[nexZone][0]),max(zones[startZone][1],zones[nexZone][1]))
        obstacles = assign_obstacles_to_zone(
            data,
            float(min(zones[int(startZone)][0], zones[int(nexZone)][0])),
            float(min(zones[int(startZone)][1], zones[int(nexZone)][1])),
            float(min(zones[int(startZone)][2], zones[int(nexZone)][2])),
            float(max(zones[int(startZone)][3], zones[int(nexZone)][3])),
            float(max(zones[int(startZone)][4], zones[int(nexZone)][4])),
            float(max(zones[int(startZone)][5], zones[int(nexZone)][5]))
        )
        episodetime,newPath, done =RRT3D(start,goal,zones[startZone],zones[goalZone],obstacles)
        T += episodetime
        path += newPath
        if not done:
            return T,path, 0, iteration_count
    with open('path3D.txt', 'w') as file:
        json.dump(path, file)
    return T, newPath, 1, iteration_count


def simulate3D_Robot(zones,policy,obstacles,start,startZone,goal,goalZone):
    nexZone=int(startZone)
    goalZone=int(goalZone)
    T=0
    print("start of the robot",start)
    print("goal of the robot",goal)
    joint_value_list=[]
    path=[]
    while nexZone != goalZone:
        nexZone=int(policy[startZone])
        print("This is the current Zone",nexZone)
        #nexZone= int(policy[startZone])
        print("they are not in the same space!")
        SubGoal= generate_safe_sub_goals_box(zones[int(nexZone)],obstacles,goal,m=10,min_distance=5,greedy=True)
        #print("next Subgoal",SubGoal)
        #print(f"start:{start},SubGoal:{SubGoal},zones:{zones[int(startZone)],zones[int(nexZone)]},obstacles:{obstacles}")
        episodetime,newPath,joint_valuess, done= RRT3D_UR10e(start,SubGoal,zones[int(startZone)],zones[int(nexZone)],obstacles)  
        if not done:
            return T,path,joint_value_list, 0
        if newPath is not None:
            path += [newPath]
            joint_value_list +=[joint_valuess] 
        startZone=nexZone
        #nexZone=int(policy[startZone])
        start= SubGoal
        T += episodetime
        path += [newPath]
        joint_value_list +=[joint_valuess]
    if nexZone == goalZone:
        print("I reached the zone")
        episodetime,newPath,joint_valuess, done =RRT3D_UR10e(start,goal,zones[startZone],zones[goalZone],obstacles)
        #print("path",newPath)
        T += episodetime
        path += [newPath]
        joint_value_list +=[joint_valuess]
        if not done:
            return T,path,joint_value_list, 0
    #with open('path3D.txt', 'w') as file:
    #    json.dump(path, file)
    return T, path,joint_value_list, 1