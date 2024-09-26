import numpy as np
from collections import Counter
from math import pi
from sklearn.preprocessing import MinMaxScaler
import random
from scipy.stats import skewnorm
from scipy.spatial import distance
import math
import random
import matplotlib.pyplot as plt
import sys
import pathlib
import time
import math
from RRT import *
sys.path.append(str(pathlib.Path(__file__).parent))
import json
def collect_grid_based(k):
    medians=[]
    for i in range(200):
        if i % k ==0:
            medians.append(i)
    return medians
    
def detect_grid(point,k):
    x= math.floor(point[0]/k)
    y= math.floor(point[0]/k)

    return x* (200/k) + y

# def get_adjacency_grid_matrix(k):
#     matrix
#     for i in range(200/k):

#     return 

#collects medians for KD-Tree
def collect_medians_of_splits(data, max_depth, depth=1, medians=None):
    
    if medians is None:
        medians = []

    if depth > max_depth or len(data) <= 1:
        return medians

    x_values = [point[0] for point in data]
    median_x = np.median(x_values)
    medians.append(median_x)

    # [print(point[0], point[1]) for point in data]

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
   
   if depth ==3:
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
   
   if depth ==4:
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
        return Zg
   

def creat_zones(medians,depth):
    if depth == 2:
        Z0=[0,0,medians[0],medians[1]] #[x0, y0, x1, y1]
        Z1=[0,medians[1],medians[0],200] 
        Z2=[medians[0],0,200,medians[2]]
        Z3=[medians[0],medians[2],200,200]
        zones = [Z0, Z1, Z2, Z3]
        Map = np.ones((2**depth, 1))
        return Map, zones
    
    if depth == 3:
        Z0=[0,0,medians[2],medians[1]]
        Z1=[0,medians[1],medians[3],200]
        Z2=[medians[2],0,medians[0],medians[1]]
        Z3=[medians[3],medians[1],medians[0],200]
        Z4=[medians[0],0,medians[5],medians[4]]
        Z5=[medians[0],medians[4],medians[6],200]
        Z6=[medians[5],0,200,medians[4]]
        Z7=[medians[6],medians[4],200,200]
        zones = [Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7]
        Map = np.ones((2**depth, 1))
        return Map, zones
    
    if depth == 4:
        Z0=[0,0,medians[2],medians[3]]
        Z1=[0,medians[3],medians[2],medians[1]]
        Z2=[0,medians[1],medians[5],medians[6]]
        Z3=[0,medians[6],medians[5],200]
        Z4=[medians[2],0,medians[0],medians[4]]
        Z5=[medians[2],0,medians[0],medians[1]]
        Z6=[medians[5],medians[1],medians[0],medians[7]]
        Z7=[medians[5],medians[7],medians[0],200]
        Z8=[medians[0],0,medians[9],medians[10]]
        Z9=[medians[0],medians[10],medians[9],medians[8]]
        Z10=[medians[0],medians[8],medians[12],medians[13]]
        Z11=[medians[0],medians[13],medians[12],200]
        Z12=[medians[9],0,200,medians[11]]
        Z13=[medians[9],medians[11],200,medians[8]]
        Z14=[medians[12],medians[8],200,medians[14]]
        Z15=[medians[12],medians[14],200,200]
        zones = [Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7,Z8,Z9,Z10,Z11,Z12,Z13,Z14,Z15]
        Map = np.ones((2**depth, 1))
        return Map, zones


def get_adjacency_matrix(zones,depth):

    if depth == 2:
        Z0, Z1, Z2, Z3 = zones
        #Actions
        adjacency_matrix = np.zeros((4,4))
        adjacency_matrix[0,1]=1
        adjacency_matrix[1,0]=1
        adjacency_matrix[2,3]=1
        adjacency_matrix[3,2]=1
        adjacency_matrix[0,2]=1
        adjacency_matrix[2,0]=1
        adjacency_matrix[1,3]=1
        adjacency_matrix[3,1]=1
        if Z1[1]>Z3[1]:
            adjacency_matrix[0,3]=1
            adjacency_matrix[3,0]=1
        else:
            adjacency_matrix[1,2]=1
            adjacency_matrix[2,1]=1
        #print(adjacency_matrix)
        return adjacency_matrix

    if depth == 3:
        Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7 = zones
        #Actions
        adjacency_matrix = np.zeros((8,8))
        for i in range(6):
            adjacency_matrix[i,i+2]=1
            adjacency_matrix[i+2,i]=1
        for i in range(0,7,2):
            adjacency_matrix[i,i+1]=1
            adjacency_matrix[i+1,i]=1
        
        if Z3[0] < Z0[2]:
            adjacency_matrix[0,3]=1
            adjacency_matrix[3,0]=1
        else:
            adjacency_matrix[1,2]=1
            adjacency_matrix[2,1]=1
        
        if Z7[0] < Z4[2]:
            adjacency_matrix[4,7]=1
            adjacency_matrix[7,4]=1
        else:
            adjacency_matrix[5,6]=1
            adjacency_matrix[6,5]=1
   
        if Z3[1] < Z4[3]:
            adjacency_matrix[3,4]=1
            adjacency_matrix[4,3]=1
        else:
            adjacency_matrix[2,5]=1
            adjacency_matrix[5,2]=1

        #print(adjacency_matrix)
        return adjacency_matrix
    
    if depth == 4:
        Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7, Z8,Z9,Z10,Z11,Z12,Z13,Z14,Z15 = zones
        #Actions
        adjacency_matrix = np.zeros((16,16))
        for i in range(12):
            adjacency_matrix[i,i+4]=1
            adjacency_matrix[i+4,i]=1
        for i in range(0,16,4):
            adjacency_matrix[i,i+1]=1
            adjacency_matrix[i+1,i]=1

            adjacency_matrix[i+1,i+2]=1
            adjacency_matrix[i+2,i+1]=1

            adjacency_matrix[i+2,i+3]=1
            adjacency_matrix[i+3,i+2]=1
    #############  Left  ################
        if Z5[1] < Z1[1]:
            adjacency_matrix[0,5]=1
            adjacency_matrix[5,0]=1
        else:
            adjacency_matrix[1,4]=1
            adjacency_matrix[4,1]=1
        if Z7[1] < Z3[1]:
            adjacency_matrix[2,7]=1
            adjacency_matrix[7,2]=1
        else:
            adjacency_matrix[3,6]=1
            adjacency_matrix[6,3]=1
        if Z6[0] < Z5[0]:
            adjacency_matrix[1,6]=1
            adjacency_matrix[6,1]=1
        else:
            adjacency_matrix[2,5]=1
            adjacency_matrix[5,2]=1
    ############ Right ###################
        if Z13[1] < Z9[1]:
            adjacency_matrix[8,13]=1
            adjacency_matrix[13,8]=1
        else:
            adjacency_matrix[9,12]=1
            adjacency_matrix[12,9]=1
        if Z15[1] < Z11[1]:
            adjacency_matrix[10,15]=1
            adjacency_matrix[15,10]=1
        else:
            adjacency_matrix[11,14]=1
            adjacency_matrix[14,11]=1
        if Z14[0] < Z13[0]:
            adjacency_matrix[9,14]=1
            adjacency_matrix[14,9]=1
        else:
            adjacency_matrix[10,13]=1
            adjacency_matrix[13,10]=1
    ############  L|R  ###################
        if Z4[3] < Z9[1]:
            adjacency_matrix[8,5]=1
            adjacency_matrix[5,8]=1
        else:
            adjacency_matrix[9,4]=1
            adjacency_matrix[4,9]=1
        if Z5[3] < Z10[1]:
            adjacency_matrix[6,9]=1
            adjacency_matrix[9,6]=1
        else:
            adjacency_matrix[5,10]=1
            adjacency_matrix[10,5]=1
        if Z6[3] < Z11[1]:
            adjacency_matrix[7,10]=1
            adjacency_matrix[10,7]=1
        else:
            adjacency_matrix[6,11]=1
            adjacency_matrix[11,6]=1
        # print(adjacency_matrix)
        return adjacency_matrix


#Calculates density of zones
def get_DZ(data,medians,zones,depth,normalized=True):

    if depth == 2:
        Z0, Z1, Z2, Z3 = zones
        L=[]
        for i in range(len(data)):
            L+=[detect_zone(data[i], medians,depth=2)]
        count = Counter(L)
        DZ = []
        for i in range(4):
            Z_i = eval(f'Z{i}')  # Dynamically access Z_i
            denominator = (Z_i[2] - Z_i[0]) * (Z_i[3] - Z_i[1])
            DZ_i =  10 *count[i]* pi / denominator
            DZ.append(DZ_i)
        
        scaler = MinMaxScaler()
        if normalized:
            DZ = scaler.fit_transform(np.array(DZ).reshape(-1, 1)).flatten()
        return DZ  

    if depth == 3:
        Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7 = zones
        L=[]
        for i in range(len(data)):
            L+=[detect_zone(data[i], medians,depth=3)]
        count = Counter(L)
        DZ = []
        for i in range(8):
            Z_i = eval(f'Z{i}')  # Dynamically access Z_i
            denominator = (Z_i[2] - Z_i[0]) * (Z_i[3] - Z_i[1])
            DZ_i =  10 *count[i]* pi / denominator
            DZ.append(DZ_i)
        
        scaler = MinMaxScaler()
        DZ_normalized = scaler.fit_transform(np.array(DZ).reshape(-1, 1)).flatten()
        return DZ_normalized   

    if depth == 4:
        Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7,Z8,Z9,Z10,Z11,Z12,Z13,Z14,Z15 = zones
        L=[]
        for i in range(len(data)):
            #print(i)
            L+=[detect_zone(data[i], medians,depth=4)]
        #print(L)
        count = Counter(L)
        DZ = []
        for i in range(16):
            Z_i = eval(f'Z{i}')  # Dynamically access Z_i
            denominator = (Z_i[2] - Z_i[0]) * (Z_i[3] - Z_i[1])
            DZ_i =  10 *count[i]* pi / denominator
            DZ.append(DZ_i)
        scaler = MinMaxScaler()
        DZ_normalized = scaler.fit_transform(np.array(DZ).reshape(-1, 1)).flatten()
        #DZ_normalized=1
        return DZ_normalized

def environment(allstateActions,state,newstate, goal_zone):
    #print(2)
    if allstateActions[state,newstate] != -1000:
        if newstate == goal_zone:
            reward= +1000
        else:
            reward= allstateActions[state,newstate] -1e3
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
    num_states = 2**max_depth
    Q = np.zeros((num_states, num_states))
    policy = np.zeros(num_states)
    Reward=[]
    for i in range(num_states):
        policy[i] = random.randint(0, num_states-1)
    for episode in range(episodes):
        state = random.randint(0, num_states-1)
        Rwr=0
        while True:
            #print(1)
            rnd = random.random()
            if rnd > epsilon:
                A = int(policy[state])
            else:
                A = int(random.randint(0, num_states-1))
            newState, reward = environment(allStateActions, state, A, goal)
            Q[state, newState] = Q[state, newState]+ alpha*(reward + gamma*max(Q[newState]) - Q[state, newState])
            state = newState
            policy = policyImprovementUsing(Q)
            Rwr += reward
            if(newState==goal):
                break
        Reward +=[Rwr]
    return Q, policy,Reward

def distances(zones,goal,normalized=True):
    distances=[]
    for zone in zones:
        # Assuming each zone is represented as a rectangle with (xmin, ymin, xmax, ymax)
        center_x = (zone[0] + zone[2]) / 2
        center_y = (zone[1] + zone[3]) / 2
        # Calculate Euclidean distance from goal to the center of the zone
        dist = distance.euclidean(goal, [center_x, center_y])
        distances.append(dist)
    if normalized:
        return [distan/max(distances) for distan in distances]
    
    return distances

def setReward(adjacency_matrix, DZ_normalized,distances_norm,depth):

    if depth == 2:
        allStateActions = np.full((4, 4), -1000,dtype=float)
        #Obtain allstateActions
        for i in range(len(allStateActions)):
            for j in range(len(allStateActions)):
                if adjacency_matrix[i,j]==1:
                    allStateActions[i,j]= -10* DZ_normalized[j] #-0.5*distances_norm[j]

        return allStateActions
    
    if depth == 3:
        allStateActions = np.full((8, 8), -1000,dtype=float)
        #Obtain allstateActions
        for i in range(len(allStateActions)):
            for j in range(len(allStateActions)):
                if adjacency_matrix[i,j]==1:
                    allStateActions[i,j]= - 0* DZ_normalized[j] - 1e10 *distances_norm[j]
        return allStateActions

    if depth == 4:
        allStateActions = np.full((16, 16), -1000,dtype=float)
        #Obtain allstateActions
        for i in range(len(allStateActions)):
            for j in range(len(allStateActions)):
                if adjacency_matrix[i,j]==1:
                    
                    allStateActions[i,j]= -0* DZ_normalized[j] - 1e5*distances_norm[j]
        return allStateActions
    
def get_final_policy(depth,Q,policy):
    numStates = 2**(depth)
    for s in range(0, numStates):
        QQ = Q[s]
        mx = max(QQ)
        idx = [i if (QQ[i] == mx) else 0 for i in range(len(QQ))]
        idx_arr = np.array(idx)
        policy[s] = choose_random_index_non_zero(idx_arr)
    return policy

def create_obstalces(data):
    data_with_ones = np.hstack((data, np.ones((data.shape[0], 1))))
    obstacle_list = [tuple(row) for row in data_with_ones]
    return obstacle_list

def ZoneConnectivity(zones,obstacle,adjacency_matrix,gammaC):

    num_zones=len(zones)
    for i in range(num_zones):
        for j in range(num_zones):
            if adjacency_matrix[i,j] == 1:
                #print(f"zone {i} and zone {j}")
                if not is_accessible(zones[i],zones[j],obstacle,gammaC):
                    #print("they are not accessible")
                    adjacency_matrix[i,j]=0
                    adjacency_matrix[j,i]=0
    
    return adjacency_matrix


def is_accessible(zone_i, zone_j, obstacles, gamma_C, delta=1):
    """
    Determines if two zones are accessible to each other.
    """
    
    # Determine the common border and get the obstacle points along it
    if is_vertical_border(zone_i, zone_j):
    #    print("vertical")
        common_border = [(x, y) for x, y in obstacles if is_within_vertical_border(zone_i, zone_j, x, y, delta)]
        common_border.sort(key=lambda p: p[1])  # Sort by y-coordinate
    else:
    #    print("horizontal")
        common_border = [(x, y) for x, y in obstacles if is_within_horizontal_border(zone_i, zone_j, x, y, delta)]
        common_border.sort(key=lambda p: p[0])  # Sort by x-coordinate
    #print(len(common_border))
    if len(common_border) <3:
    #    print("None")
        return True
    # Check maximum gap between obstacles on the common border
    max_gap = 0
    for k in range(len(common_border) - 1):
        gap = np.linalg.norm(np.array(common_border[k]) - np.array(common_border[k + 1]))
        #print("gap",gap)
        if gap > max_gap:
            max_gap = gap
    #print(f"the maximum gap is {max_gap}")
    return max_gap >= gamma_C

def has_common_border(zone_i, zone_j):
    """
    Checks if two zones have a common border.
    """
    return (zone_i[2] == zone_j[0] or zone_i[0] == zone_j[2] or
            zone_i[3] == zone_j[1] or zone_i[1] == zone_j[3])

def is_vertical_border(zone_i, zone_j):
    """
    Checks if the common border between two zones is vertical.
    """
    return zone_i[2] == zone_j[0] or zone_i[0] == zone_j[2]

def is_within_vertical_border(zone_i, zone_j, x, y, delta):
    """
    Checks if a point is within the vertical common border of two zones.
    """
    common_x_border = zone_i[2] if zone_i[2] == zone_j[0] else zone_i[0]
    return (common_x_border - delta <= x <= common_x_border + delta and
            min(zone_i[1], zone_j[1]) <= y <= max(zone_i[3], zone_j[3]))

def is_within_horizontal_border(zone_i, zone_j, x, y, delta):
    """
    Checks if a point is within the horizontal common border of two zones.
    """
    common_y_border = zone_i[3] if zone_i[3] == zone_j[1] else zone_i[1]
    return (common_y_border - delta <= y <= common_y_border + delta and
            min(zone_i[0], zone_j[0]) <= x <= max(zone_i[2], zone_j[2]))

def plot_data(data, medians_collected, max_depth):
    fig, ax = plt.subplots()
    for i in range(len(data)):
        x_scaled = data[:, 0]
        y_scaled = data[:, 1]
        circle = plt.Circle((x_scaled[i], y_scaled[i]), 1, color='green')
        ax.add_patch(circle)
        
    medians_collected = collect_medians_of_splits(data, max_depth)
    plt.scatter(x_scaled, y_scaled)
    plot(data, medians_collected, max_depth)

def plot(data, medians, depth):

    plt.scatter(data[:, 0], data[:, 1])
    plt.axis([0, 200, 0, 200])
    if depth == 2:
        plt.plot([medians[0], medians[0]], [0, 200])
        plt.plot([0, medians[0]], [medians[1], medians[1]])
        plt.plot([medians[0], 200], [medians[2], medians[2]])
    elif depth == 3:
        plt.plot([medians[0], medians[0]], [0, 200])
        plt.plot([0, medians[0]], [medians[1], medians[1]])
        plt.plot([medians[0], 200], [medians[4], medians[4]])
        plt.plot([medians[2], medians[2]], [0, medians[1]])
        plt.plot([medians[3], medians[3]], [medians[1], 200])
        plt.plot([medians[5], medians[5]], [0, medians[4]])
        plt.plot([medians[6], medians[6]], [medians[4], 200])
    elif depth == 4:
        #plt.plot([medians[0], medians[0]], [0, 200])  #vert1
        #plt.plot([0, medians[0]], [medians[1], medians[1]]) #horz1 (left)
        #plt.plot([medians[0], 200], [medians[4], medians[4]]) #horz2 (right)
        #plt.plot([medians[2], medians[2]], [0, medians[1]]) #vertical2 (left-down)
        #plt.plot([medians[3], medians[3]], [medians[1], 200]) #vertical3(left-up)
        #plt.plot([medians[5], medians[5]], [0, medians[4]]) 
        #plt.plot([medians[6], medians[6]], [medians[4], 200])
        #plt.plot([medians[2], medians[2]], [0, medians[1]])
        #plt.plot([medians[3], medians[3]], [medians[1], 200])
        #plt.plot([medians[5], medians[5]], [0, medians[4]])
        #plt.plot([medians[6], medians[6]], [medians[4], 200])
        # Plotting for depth 3
        plt.plot([medians[0], medians[0]], [0, 200], 'r') #0
        plt.plot([0, medians[0]], [medians[1], medians[1]], 'g') #1
        plt.plot([medians[2], medians[2]], [0, medians[1]], 'b') #2
        plt.plot([0, medians[2]],[medians[3], medians[3]], 'b') #3
        plt.plot([medians[2],medians[0]],[medians[4], medians[4]], 'b') #4
        plt.plot([medians[5], medians[5]], [medians[1], 200], 'b')#5
        plt.plot([0, medians[5]], [medians[6], medians[6]], 'b') #6
        plt.plot([medians[5],medians[0]], [medians[7], medians[7]], 'b') #7
        plt.plot([medians[0],200], [medians[8], medians[8]], 'b') #8
        plt.plot([medians[9],medians[9]], [0, medians[8]], 'b') #9
        plt.plot([medians[0], medians[9]],[medians[10], medians[10]], 'b') #10
        plt.plot([medians[9], 200], [medians[11], medians[11]], 'b') #11
        plt.plot([medians[12], medians[12]], [medians[8], 200], 'b') #12
        plt.plot([medians[0], medians[12]], [medians[13], medians[13]], 'b') #13
        plt.plot([medians[12], 200], [medians[14], medians[14]], 'b') #14

def is_point_too_close_to_obstacles(point, obstacle_list, min_distance=5):
    radius= 1 #radius of obstacles! 
    for obstacle in obstacle_list:
        ox, oy = obstacle
        distance_po = np.sqrt((point[0] - ox)**2 + (point[1] - oy)**2)
        if distance_po < (radius + min_distance):
            return True
    return False

def generate_safe_sub_goals(Z_next, obstacle_list,goal, m, min_distance=10,greedy=True):
    safe_sub_goals = []
    while len(safe_sub_goals) < m:
        candidate_point = (random.uniform(Z_next[0], Z_next[2]), random.uniform(Z_next[1], Z_next[3]))
        if not is_point_too_close_to_obstacles(candidate_point, obstacle_list, min_distance):
            safe_sub_goals.append(candidate_point)
    #choose one of them
    if greedy:
        # Choose the subgoal nearest to the goal
        safe_sub_goals.sort(key=lambda point: np.linalg.norm(np.array(point) - np.array(goal)))
        return safe_sub_goals[0]  
    
    return random.choice(safe_sub_goals)

def assign_obstacles_to_zone(data, xmin, ymin, xmax, ymax):

    obstacles_in_zone = []

    for obstacle in data:
        x, y = obstacle
        if xmin <= x <= xmax and ymin <= y <= ymax:
            obstacles_in_zone.append(obstacle)
    
    return obstacles_in_zone



def simulate2D(zones,policy,data,start,startZone,goal,goalZone):
    startZone = int(startZone)
    goalZone= int(goalZone)
    #nexZone=int(policy[startZone])
    nexZone = startZone
    T=0
    path=[]
    safe_sub_goal=[]
    print("Next Zone is",nexZone)
    while nexZone != goalZone:
        nexZone=int(policy[startZone])
        print("Next Zone is",nexZone)
        
        #obstacles=assign_obstacles_to_zone(data,min(zones[startZone][0],zones[nexZone][0]),min(zones[startZone][1],zones[nexZone][1]),max(zones[startZone][0],zones[nexZone][0]),max(zones[startZone][1],zones[nexZone][1]))       
        obstacles = assign_obstacles_to_zone(
            data,
            float(min(zones[startZone][0], zones[nexZone][0])),
            float(min(zones[startZone][1], zones[nexZone][1])),
            float(max(zones[startZone][2], zones[nexZone][2])),
            float(max(zones[startZone][3], zones[nexZone][3]))
        )
        SubGoal= generate_safe_sub_goals(zones[nexZone],obstacles,goal,m=10,min_distance=10,greedy=True)
        print("next Subgoal",SubGoal)
        safe_sub_goal.append(SubGoal)
        episodetime,newPath, done,iteration_count= RRT2D(start,SubGoal,zones[startZone],zones[nexZone],obstacles)  
        if not done:
            return T,path, 0, iteration_count,safe_sub_goal
        if newPath is not None:
            path += [newPath]
        
        startZone=nexZone
        #nexZone=int(policy[startZone])
        start= SubGoal
        T += episodetime
        path += [newPath]
    if nexZone == goalZone:
        #obstacles=assign_obstacles_to_zone(data,min(zones[startZone][0],zones[nexZone][0]),min(zones[startZone][1],zones[nexZone][1]),max(zones[startZone][0],zones[nexZone][0]),max(zones[startZone][1],zones[nexZone][1]))
        zones[startZone] = list(map(float, zones[startZone]))
        zones[nexZone] = list(map(float, zones[nexZone]))
        #print("index of start zone",zones[startZone])
        obstacles = assign_obstacles_to_zone(
            data,
            float(min(zones[startZone][0], zones[nexZone][0])),
            float(min(zones[startZone][1], zones[nexZone][1])),
            float(max(zones[startZone][2], zones[nexZone][2])),
            float(max(zones[startZone][3], zones[nexZone][3]))
        )
        episodetime,newPath, done, iteration_count =RRT2D(start,goal,zones[startZone],zones[goalZone],obstacles)
        T += episodetime
        path += [newPath]
        if not done:
            return T,path, 0, iteration_count,safe_sub_goal
    with open('path.txt', 'w') as file:
        json.dump(path, file) 
    if T > 0.4:
        return T, path, 0 ,iteration_count,safe_sub_goal
    return T, path, 1 ,iteration_count,safe_sub_goal
        


