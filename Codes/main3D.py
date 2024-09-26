from QLearning3D import *
import time
np.random.seed(42)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

start=[0,0,0]
goal=[199,199,199]
data = np.random.rand(200, 3) * 200
depth2D=2
depthZ=1
T_RRT=[]
T_RL=[]
SR=[]
medians2D= collect_medians_of_splits(data, depth2D)
print("medians 2D",medians2D)
mediansZ= collect_medians_z(data,depthZ,depth=1,medians_z=None)
print("medians 3D",mediansZ)
AllMedians= medians2D +mediansZ
startZone=Final_zone(start,medians2D,mediansZ,depth2D,depthZ)
goalZone=Final_zone(goal,medians2D,mediansZ,depth2D,depthZ)
print("startZone, goalZone",startZone,goalZone)
zones= create_zone(data,depth2D,depthZ,boundry=200)
adjacency_matrix= get_adjacency_matrix(zones,depth2D)
#adjacency_matrix=ZoneConnectivity(zones, data, adjacency_matrix, gammaC=1)
print("here is the adjacency matrix",adjacency_matrix)
DofZ= get_DofZ(data,medians2D,mediansZ,depth2D,depthZ,zones)
dist= distances(zones,goal,normalized=True)
AllStatesActions= setReward(adjacency_matrix, DofZ, dist, depth2D)
gamma=0.9
episodes=2000
epsilon=0.9
alpha=0.1
startRL=time.time()
Q, policy = train(depth2D, episodes, alpha, gamma, epsilon, AllStatesActions, goalZone)
endRL=time.time()
policy = get_final_policy(depth2D,Q,policy)
print("policy is here",policy)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def check_success(path, goal, time, tolerance=1e-2):
    if not path:
        return 0
    
    final_state = path[-1]
    print("final state",final_state)
    check= np.allclose(final_state, goal, atol=tolerance)
    print("check",check)
    if check:
        if time > 4:
            print("Hi")
            return 0
        else:
            print("hi")
            return 1
    return 0

for i in range(100):
    Time, path,done,iteration_count = simulate3D(zones,policy,data,start,startZone,goal,goalZone)

    T_RRT +=[Time]
    SR +=[check_success(path,goal,Time,tolerance=1e-2)]
    # Extract path coordinates
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]
    z_coords = [point[2] for point in path]
    ax.plot(x_coords, y_coords, z_coords, label=f'Path {i+1}')


ax.scatter(start[0], start[1], start[2], c='red', marker='o', label='Start')
ax.scatter(goal[0], goal[1], goal[2], c='green', marker='x', label='Goal')
# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set title and legend
ax.set_title('3D Path Planning')
ax.legend()

# Show plot
plt.show()

print(SR)
print(f"succes rate is {SR.count(1)/len(SR)*100}%")
print(f" RL Time",endRL-startRL)
print(f" mean RRT Time",sum(T_RRT)/len(T_RRT))
print(f"number of iteration is {iteration_count}")