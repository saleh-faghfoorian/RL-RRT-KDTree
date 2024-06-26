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
sys.path.append(str(pathlib.Path(__file__).parent))
def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.hypot(dx, dy)
        le += d
    return le
def get_target_point(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.hypot(dx, dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

    return [x, y, ti]


def line_collision_check(first, second, obstacleList):
    # Line Equation

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.hypot(a, b))
        if d <= size:
            return False

    return True  # OK


def path_smoothing(path, max_iter, obstacle_list):
    le = get_path_length(path)

    for i in range(max_iter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not line_collision_check(first, second, obstacle_list):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = get_path_length(path)

    return path

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



    
    #plt.plot([medians[0], 200], [medians[4], medians[4]], 'g') 
    #plt.plot([medians[2], medians[2]], [0, medians[1]], 'b')
    #plt.plot([medians[3], medians[3]], [medians[1], 200], 'b')
    #plt.plot([medians[5], medians[5]], [0, medians[4]], 'b')
    #plt.plot([medians[6], medians[6]], [medians[4], 200], 'b')
        # Additional plots for depth 4
    #plt.plot([0, medians[2]], [medians[7], medians[7]], 'y')
    #plt.plot([medians[2], medians[0]], [medians[8], medians[8]], 'y')
    #plt.plot([medians[3], medians[0]], [medians[9], medians[9]], 'y')
    #plt.plot([medians[3], 200], [medians[10], medians[10]], 'y')
    #plt.plot([0, medians[5]], [medians[11], medians[11]], 'y')
    #plt.plot([medians[5], medians[0]], [medians[12], medians[12]], 'y')
    #plt.plot([medians[6], medians[0]], [medians[13], medians[13]], 'y')
    #plt.plot([medians[6], 200], [medians[14], medians[14]], 'y')

#def detect_zone(goal, medians_collected):
#  if goal[0]>medians_collected[0]:
#    if goal[1]> medians_collected[4]:
#      if goal[0]>medians_collected[6]:
#        Zg=7
#      else:
#        Zg=5
#    else:
#      if goal[0]>medians_collected[5]:
#        Zg=6
#      else:
#        Zg=4
#  else:
#    if goal[1]>medians_collected[1]:
#      if goal[0]>medians_collected[3]:
#        Zg=3
#      else:
#        Zg=1
#    else:
#      if goal[0]>medians_collected[2]:
#        Zg=2
#      else:
#        Zg=0
#  return(Zg)

def detect_zone(goal, medians_collected):
  Zg=-1
  # Right
  if goal[0]>medians_collected[0]:
    if goal[1]> medians_collected[8]:
      if goal[0]>medians_collected[12]:
        if goal[1]>medians_collected[14]:
          Zg=15
        else:
          Zg=14
      else:
        if goal[1]>medians_collected[13]:
          Zg=11
        else:
          Zg=10
    if goal[1]< medians_collected[8]:
      if goal[0]>medians_collected[9]:
        if goal[1]>medians_collected[11]:
          Zg=13
        else:
          Zg=12
      else:
        if goal[1]>medians_collected[10]:
          Zg=9
        else:
          Zg=8
  # Left        
  if goal[0]<medians_collected[0]:
    if goal[1]> medians_collected[1]:
      if goal[0]>medians_collected[5]:
        if goal[1]>medians_collected[7]:
          Zg=7
        else:
          Zg=6
      else:
        if goal[1]>medians_collected[6]:
          Zg=3
        else:
          Zg=2
    if goal[1]< medians_collected[1]:
      if goal[0]>medians_collected[2]:
        if goal[1]>medians_collected[4]:
          Zg=5
        else:
          Zg=4
      else:
        if goal[1]>medians_collected[3]:
          Zg=1
        else:
          Zg=0 
  return(Zg)


#def setRewards(Map,adjacency_matrix, DZ_normalized,distances_norm):
#    allStateActions = np.full((8, 8), -1000,dtype=float)
    #Obtain allstateActions
#    for i in range(len(allStateActions)):
#      for j in range(len(allStateActions)):
#        if adjacency_matrix[i,j]==1:
#          allStateActions[i,j]= -10* DZ_normalized[j] #-0.5*distances_norm[j]


#    return allStateActions

def setRewards(Map,adjacency_matrix, DZ_normalized,distances_norm):
    allStateActions = np.full((16, 16), -1000,dtype=float)
    #Obtain allstateActions
    for i in range(len(allStateActions)):
      for j in range(len(allStateActions)):
        if adjacency_matrix[i,j]==1:
          allStateActions[i,j]= -10* DZ_normalized[j] -0.5*distances_norm[j]


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


#def get_adjacency_matrix(zones):
#    Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7 = zones
    #Actions
#    adjacency_matrix = np.zeros((8,8))
#    for i in range(6):
#        adjacency_matrix[i,i+2]=1
#        adjacency_matrix[i+2,i]=1
#    for i in range(0,7,2):
#        adjacency_matrix[i,i+1]=1
#        adjacency_matrix[i+1,i]=1
#    if Z3[0] < Z0[2]:
#        adjacency_matrix[0,3]=1
#        adjacency_matrix[3,0]=1
#    else:
#        adjacency_matrix[1,2]=1
#        adjacency_matrix[2,1]=1
#    if Z7[0] < Z4[2]:
#        adjacency_matrix[4,7]=1
#        adjacency_matrix[7,4]=1
#    else:
#        adjacency_matrix[5,6]=1
#        adjacency_matrix[6,5]=1
#    if Z3[1] < Z4[3]:
#        adjacency_matrix[3,4]=1
#        adjacency_matrix[4,3]=1
#    else:
#        adjacency_matrix[2,5]=1
#        adjacency_matrix[5,2]=1

    # print(adjacency_matrix)
#    return adjacency_matrix

def get_adjacency_matrix(zones):
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
    ##############  Left  ################
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
    ############# Right ###################
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
    #############  L|R  ###################
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



#def get_DZ(data, medians_collected, zones):
#    Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7 = zones
#    L=[]
#    for i in range(len(data)):
#        L+=[detect_zone(data[i], medians_collected)]
#    count = Counter(L)
#    DZ = []
#    for i in range(8):
#        Z_i = eval(f'Z{i}')  # Dynamically access Z_i
#        denominator = (Z_i[2] - Z_i[0]) * (Z_i[3] - Z_i[1])
#        DZ_i =  10 *count[i]* pi / denominator
#        DZ.append(DZ_i)
#    scaler = MinMaxScaler()
#    DZ_normalized = scaler.fit_transform(np.array(DZ).reshape(-1, 1)).flatten()
#    return DZ_normalized

def get_DZ(data, medians_collected, zones):
    #print("HIIssssssIIIII")
    Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7,Z8,Z9,Z10,Z11,Z12,Z13,Z14,Z15 = zones
    L=[]
    for i in range(len(data)):
        #print(i)
        L+=[detect_zone(data[i], medians_collected)]
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


def train(max_depth, episodes, alpha, gamma, epsilon, allStateActions, goal):
    num_states = 2**max_depth
    Q = np.zeros((num_states, num_states))
    policy = np.zeros(num_states)
    for i in range(num_states):
        policy[i] = random.randint(0, num_states-1)
    for episode in range(episodes):
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



def generate_data(a1,a2, obstacles):
    x = skewnorm.rvs(a=a1, loc=200, size=obstacles)
    y = skewnorm.rvs(a=a2, loc=200, size=obstacles)
    x_scaled = (x - x.min()) / (x.max() - x.min()) * 200
    y_scaled = (y - y.min()) / (y.max() - y.min()) * 200
    data = np.zeros((len(x), 2))
    for i in range(len(x)):
        data[i] = [x_scaled[i], y_scaled[i]]
    return data


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


#def create_zones(medians_collected, max_depth):
#    Z0=[0,0,medians_collected[2],medians_collected[1]]
#    Z1=[0,medians_collected[1],medians_collected[3],200]
#    Z2=[medians_collected[2],0,medians_collected[0],medians_collected[1]]
#    Z3=[medians_collected[3],medians_collected[1],medians_collected[0],200]
#    Z4=[medians_collected[0],0,medians_collected[5],medians_collected[4]]
#    Z5=[medians_collected[0],medians_collected[4],medians_collected[6],200]
#    Z6=[medians_collected[5],0,200,medians_collected[4]]
#    Z7=[medians_collected[6],medians_collected[4],200,200]
#    zones = [Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7]
#    Map = np.ones((2**max_depth, 1))
#    return Map, zones

def create_zones(medians_collected, max_depth):
    Z0=[0,0,medians_collected[2],medians_collected[3]]
    Z1=[0,medians_collected[3],medians_collected[2],medians_collected[1]]
    Z2=[0,medians_collected[1],medians_collected[5],medians_collected[6]]
    Z3=[0,medians_collected[6],medians_collected[5],200]
    Z4=[medians_collected[2],0,medians_collected[0],medians_collected[4]]
    Z5=[medians_collected[2],0,medians_collected[0],medians_collected[1]]
    Z6=[medians_collected[5],medians_collected[1],medians_collected[0],medians_collected[7]]
    Z7=[medians_collected[5],medians_collected[7],medians_collected[0],200]
    Z8=[medians_collected[0],0,medians_collected[9],medians_collected[10]]
    Z9=[medians_collected[0],medians_collected[10],medians_collected[9],medians_collected[8]]
    Z10=[medians_collected[0],medians_collected[8],medians_collected[12],medians_collected[13]]
    Z11=[medians_collected[0],medians_collected[13],medians_collected[12],200]
    Z12=[medians_collected[9],0,200,medians_collected[11]]
    Z13=[medians_collected[9],medians_collected[11],200,medians_collected[8]]
    Z14=[medians_collected[12],medians_collected[8],200,medians_collected[14]]
    Z15=[medians_collected[12],medians_collected[14],200,200]
    zones = [Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7,Z8,Z9,Z10,Z11,Z12,Z13,Z14,Z15]
    Map = np.ones((2**max_depth, 1))
    return Map, zones

def create_obstalces(data):
    data_with_ones = np.hstack((data, np.ones((data.shape[0], 1))))
    obstacle_list = [tuple(row) for row in data_with_ones]
    return obstacle_list


def get_final_policy(max_depth, Q, policy):
    numStates = 2**(max_depth)
    for s in range(0, numStates):
        QQ = Q[s]
        mx = max(QQ)
        idx = [i if (QQ[i] == mx) else 0 for i in range(len(QQ))]
        idx_arr = np.array(idx)
        policy[s] = choose_random_index_non_zero(idx_arr)
    return policy

def distance_to_goal(goal, zones):
    distances = []
    for zone in zones:
        # Assuming each zone is represented as a rectangle with (xmin, ymin, xmax, ymax)
        center_x = (zone[0] + zone[2]) / 2
        center_y = (zone[1] + zone[3]) / 2
        # Calculate Euclidean distance from goal to the center of the zone
        dist = distance.euclidean(goal, [center_x, center_y])
        distances.append(dist)
    scaler = MinMaxScaler()
    distances_norm= scaler.fit_transform(np.array(distance).reshape(-1, 1)).flatten()
    return distances_norm
"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random

import matplotlib.pyplot as plt
show_animation = False
import time

class RRT:

    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])


    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=5000,
                 play_area=None,
                 robot_radius=0.0,
                 Z1=None,
                 Z2=None
                 ):
      
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius
        self.Z1 = Z1 if Z1 is not None else [0, 0, 200, 200]  # Default value if not provided
        self.Z2 = Z2 if Z2 is not None else [0, 0, 200, 200]
        self.number_of_iterations = 0

    def planning(self, animation=True):

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)
                self.number_of_iterations += 1

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(
                        final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)
        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
      if random.randint(0, 100) > self.goal_sample_rate:
          # Decide which zone to use for generating a random node
          zone = self.Z2
          rnd = self.Node(
              random.uniform(zone[0], zone[2]),  # xmin to xmax
              random.uniform(zone[1], zone[3]))  # ymin to ymax
      else:  # goal point sampling
          rnd = self.Node(self.end.x, self.end.y)
      return rnd


    def draw_graph(self, rnd=None):
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
#goal=[0,175]
#start=[150,50]
goal=[172,181.3]
#goal=[7.4,150]
start=[11,55.5]

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def is_point_too_close_to_obstacles(point, obstacle_list, min_distance=5):
  for obstacle in obstacle_list:
    ox, oy, radius = obstacle
    distance_po = np.sqrt((point[0] - ox)**2 + (point[1] - oy)**2)
    if distance_po < (radius + min_distance):
      return True
  return False
  
def generate_safe_sub_goals(Z_next, obstacle_list, m, min_distance=5):
  safe_sub_goals = []
  while len(safe_sub_goals) < m:
      candidate_point = (random.uniform(Z_next[0], Z_next[2]), random.uniform(Z_next[1], Z_next[3]))
      if not is_point_too_close_to_obstacles(candidate_point, obstacle_list, min_distance):
          safe_sub_goals.append(candidate_point)
  return safe_sub_goals



def simulate(medians_collected, zones, policy, obstacle_list, gx=goal[0], gy=goal[1], sx=start[0], sy=start[1], show_animation=False):
    Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7,Z8,Z9,Z10,Z11,Z12,Z13,Z14,Z15 = zones
    algo_start_time = time.time()
    plot_time = 0
    #print("start " + __file__)
    #current zone
    currentz = detect_zone([sx,sy], medians_collected)
    Z_current = eval(f'Z{currentz}')
    #goal zone
    goalz = detect_zone([gx,gy], medians_collected)
    Z_goal = eval(f'Z{goalz}')
    #next zone
    nextZone = int(policy[currentz])
    Z_next = eval(f'Z{nextZone}')
    #Initialize start point
    start = [sx,sy]
    # sampling in the next zone
    m = 10
    #sub_goals = [[random.uniform(Z_next[0], Z_next[2]), random.uniform(Z_next[1], Z_next[3])] for _ in range(m)]
    #sub_goal = min(sub_goals, key=lambda x: (x[0] - goal[0])**2 + (x[1] - goal[1])**2)
    safe_sub_goals = generate_safe_sub_goals(Z_next, obstacle_list, m)
    sub_goal = min(safe_sub_goals, key=lambda x: (x[0] - goal[0])**2 + (x[1] - goal[1])**2)
    #print(sub_goals)
    obstacleList = obstacle_list
    path=[]
    plt.clf()
    number_of_iterations = 0
    while nextZone != goalz:
      # ====Search Path with RRT====
      # Set Initial parameters
        rrt = RRT(
            start=start,
            goal=[sub_goal[0], sub_goal[1]],
            rand_area=[-2, 15],
            obstacle_list=obstacleList,
            # play_area=[0, 10, 0, 14]
            robot_radius=0.8,
            Z1=Z_current,
            Z2=Z_next,
            )
        #new_path = rrt.planning(animation = show_animation)
        new_path = rrt.planning(animation = show_animation)
        if new_path is not None:
            path += new_path
        number_of_iterations += rrt.number_of_iterations
        start_time = time.time()
        rrt.draw_graph()
        plot_time += time.time() - start_time
        currentz = nextZone
        Z_current = Z_next
        nextZone = int(policy[currentz])
        Z_next = eval(f'Z{nextZone}')
        start = sub_goal
        m = 10
        #sub_goals = [[random.uniform(Z_next[0], Z_next[2]), random.uniform(Z_next[1], Z_next[3])] for _ in range(m)]
        safe_sub_goals = generate_safe_sub_goals(Z_next, obstacle_list, m)
        sub_goal = min(safe_sub_goals, key=lambda x: (x[0] - goal[0])**2 + (x[1] - goal[1])**2)

        #sub_goal = min(sub_goals, key=lambda x: (x[0] - goal[0])**2 + (x[1] - goal[1])**2)
        #print(sub_goals)
    if nextZone == goalz:
        rrt = RRT(
            start=start,
            goal=[gx, gy],
            rand_area=[-2, 15],
            obstacle_list=obstacleList,
            # play_area=[0, 10, 0, 14]
            robot_radius=0.8,
            Z1=Z_current,
            Z2=Z_goal
            )
        new_path = rrt.planning(animation = show_animation)
        #maxIter = 100
        #path = path_smoothing(path, maxIter, obstacleList)
        if new_path is not None:
            path += new_path
        number_of_iterations += rrt.number_of_iterations
    maxIter = 1000
    path = path_smoothing(path, maxIter, obstacleList)
    start_time = time.time()
    rrt.draw_graph()
    plot_time += time.time() - start_time
    algo_time = time.time() - plot_time - algo_start_time
    plt.axis("equal")
    plt.axis([0, 200, 0, 200])
    plt.grid(True)
    plt.show()
    return algo_time, number_of_iterations

          # Draw final path
          #if show_animation:
          #    rrt.draw_graph()
          #    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
          #    plt.grid(True)
          #    plt.pause(0.01)  # Need for Mac
          #    plt.show()



