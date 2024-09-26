from QLearning2D import *
np.random.seed(42)
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import scipy.stats as stats
import time
T_RRT=[]
T_RL=[]
SR=[]
start=[25,25]
goal= [190,190]
number_of_points=1000


# Parameters
np.random.seed(20)
random.seed(20)
n = 1000
loc = 100
scale = 50
a_range = (-2, 2)  # Adjust this range to control the skewness

# Generate skewed data
#data = generate_skewed_data(n, loc, scale, a_range)
#data= np.random.rand(number_of_points,2)*200
number_of_points = 500

#Generate random angles between 0 and 2*pi
angles = np.random.rand(number_of_points) * 2 * np.pi

#Generate random radii with uniform distribution within the circle
radii = np.sqrt(np.random.rand(number_of_points)) * 50  # sqrt to ensure uniform distribution

#Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
x = radii * np.cos(angles) + 100  # Center x = 100
y = radii * np.sin(angles) + 100  # Center y = 100

#Combine x and y into a single array of points
data = np.column_stack((x, y))
data = np.array([])

depth=2
median=collect_medians_of_splits(data,depth)
print("medians",median)

goalZone=detect_zone(goal,median,depth)
startZone=detect_zone(start,median,depth)
print("startZone, goalZone",startZone,goalZone)
_,zones= creat_zones(median,depth)
adjMatrix=get_adjacency_matrix(zones,depth)
#print("here is the adjacency matrix",adjMatrix)
adjMatrix=ZoneConnectivity(zones,data,adjMatrix,gammaC=1)
#print("here is the uodated adjacency matrix",adjMatrix)
DofZ=get_DZ(data,median,zones,depth,normalized=False)
#print("Density of Zones",DofZ)
dist=distances(zones,goal,normalized=True)
AllStateActions=setReward(adjMatrix,DofZ,dist,depth)
gamma=0.9
episodes=1000
epsilon=0.9
alpha=0.1
print("train is started")
startRL=time.time()
Q,policy,Reward=train(depth,episodes,alpha,gamma,epsilon,AllStateActions,goalZone)
endRL=time.time()
policy=get_final_policy(depth,Q,policy)
print("policy is here",policy)

iter=[]
desired_width_px = 343
desired_height_px = 365
dpi = 100  # Dots per inch

# Convert pixels to inches
width_in = desired_width_px / dpi
height_in = desired_height_px / dpi

def check_success(path, goal, time, tolerance=1e-2):
    if not path:
        return 0
    
    final_state = path[-1]
    print("final state",final_state)
    check= np.allclose(final_state, goal, atol=tolerance)
    #print("check",check)
    if check:
        if time > 5:
            print("-------> 0")
            return 0
        else:
            print("-------> 1")
            return 1
    return 0

# Create a figure with the specified size
fig, ax = plt.subplots(figsize=(10, 10))#, dpi=dpi)
for i in range(1):
    print(f"----------------This is the {i}th iteration-----------------------")
    Time, path,done,iteration_count, safe_sub_goal= simulate2D(zones,policy,data,start,startZone,goal,goalZone)
    T_RRT +=[Time]
    path = [point for sublist in path for point in sublist]
    SR +=[check_success(path,[190,190],Time,tolerance=1e-2)]
    print("safe sub goal",safe_sub_goal)
    for target_tuple in safe_sub_goal:
        try:
            # Find the first and second occurrence of the target tuple
            first_index = path.index(target_tuple)
            second_index = path.index(target_tuple, first_index + 1)
            third_index = path.index(target_tuple, second_index + 1)
            fourth_index = path.index(target_tuple, third_index + 1)
            # Remove the elements between the two occurrences (including the first and excluding the last)
            if first_index < second_index:
                path = path[:first_index] + path[fourth_index:]
            
        except ValueError:
            # In case the tuple is not found twice in the path
            print(f"{target_tuple} does not occur twice in the path")
    print("--------------------")
    print(path)
    print("--------------------")
    
    # Extract path coordinates
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]
    #iter=calculate_path_length(path)
    #print("dist",iter)
    # Plot path
    ax.plot(x_coords, y_coords)#, label=f'Path {i+1}')

# Plot obstacles
for obstacle in data:
    circle = patches.Circle((obstacle[0], obstacle[1]), radius=1, edgecolor='black', facecolor='gray')
    ax.add_patch(circle)

print(safe_sub_goal)
# Plot start and goal
ax.scatter(start[0], start[1], c='red', marker='o')#, label='Start')
ax.scatter(goal[0], goal[1], c='green', marker='x')#, label='Goal')
# Unpack the list of tuples using zip
x_subgoals, y_subgoals = zip(*safe_sub_goal)

# Plot the subgoals with a "*" marker
#plt.scatter(x_subgoals, y_subgoals, label='Subgoals', color='red', marker='*', s=100)


# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')

# Set title and legend
ax.set_title('2D Path Planning')
ax.legend()

print("HIII")
# Show plot
plt.xlim(0, 200)
plt.ylim(0, 200)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
print(SR)
print(np.mean(iter))
print(f"succes rate is {SR.count(1)/len(SR)*100}%")
print(f" RL Time",endRL-startRL)
print(f" mean RRT Time",np.mean(T_RRT))
print(f"number of iteration is {iteration_count}")
