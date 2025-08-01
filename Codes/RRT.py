import ompl.base as ob
import ompl.geometric as og
from ompl import util as ou
import numpy as np
import time
from environment.inverse_kinematics_ur10e import inverse_kinematics
import mujoco
from gymnasium.envs.registration import register
import gymnasium

def is_state_valid_3d(state, env):
    for i in range(20):
        state = [state[0], state[1], state[2]]
        act = inverse_kinematics(np.array(state), np.eye(3))
        env.step(act)
    contact_list = []
    for i in range(env.data.ncon):
        contact = env.data.contact[i]    
        geom1 = contact.geom1
        geom2 = contact.geom2
        geom1_name = env.model.geom(geom1).name
        geom2_name = env.model.geom(geom2).name
        contact_list.append((geom1_name, geom2_name))
    if len(contact_list) == 0:
        return True
    else:
        return False

# def is_state_valid(state, obstacles, radius=1):
#     for (ox, oy) in obstacles:
#         distance = np.linalg.norm([state[0] - ox, state[1] - oy])
#         if distance < radius:  # Consider the radius of the obstacles
#             return False
#     return True

def is_point_too_close_to_obstacles(point, obstacle_list, min_distance):
    
    for obstacle in obstacle_list:
        if obstacle.point_in_rect(point, min_distance):
            return True
    return False

def is_state_valid_rect(state, obstacles, min_distance=0):

    for obs in obstacles:
        if obs.point_in_rect(state, min_distance):
            return False
        # if not is_point_too_close_to_obstacles(state, obstacles, min_distance):
        #     return True
    return True


def is_state_valid_box(state, env):
    for i in range(20):
        state = [state[0], state[1], state[2]]
        act = inverse_kinematics(np.array(state), np.eye(3))
        env.step(act)
    contact_list = []
    for i in range(env.data.ncon):
        contact = env.data.contact[i]    
        geom1 = contact.geom1
        geom2 = contact.geom2
        geom1_name = env.model.geom(geom1).name
        geom2_name = env.model.geom(geom2).name
        contact_list.append((geom1_name, geom2_name))
    if len(contact_list) == 0:
        return True
    else:
        return False


def RRT2D(start, subgoal, zone_start, zone_next, obstacles):
    space = ob.RealVectorStateSpace(2)
    
    lower_bound = [min(zone_start[0], zone_next[0]), min(zone_start[1], zone_next[1])]
    upper_bound = [max(zone_start[2], zone_next[2]), max(zone_start[3], zone_next[3])]
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, lower_bound[0])
    bounds.setLow(1, lower_bound[1])
    bounds.setHigh(0, upper_bound[0])
    bounds.setHigh(1, upper_bound[1])
    
    space.setBounds(bounds)
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(lambda state: is_state_valid_rect(state, obstacles)))
    
    start_state = ob.State(space)
    start_state()[0] = float(start[0])
    start_state()[1] = float(start[1])
    
    goal_state = ob.State(space)
    goal_state()[0] = float(subgoal[0])
    goal_state()[1] = float(subgoal[1])
    
    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start_state, goal_state)
    
    planner = og.RRT(si)
    planner.setRange(12) 
    planner.setProblemDefinition(pdef)
    planner.setup()
    startRRT=time.time()
    solved = planner.solve(2)  # Allow 10 seconds to solve
    endRRT=time.time()
    iteration_count = 1
    print("endRRT-startRRT",endRRT-startRRT)
    if endRRT- startRRT <2:
        path = pdef.getSolutionPath()
        new_path = [(state[0], state[1]) for state in path.getStates()]
        planning_time = endRRT-startRRT  # Get the planning time
        return planning_time, new_path, 1,iteration_count
    else:
        return 10, [], 0,iteration_count  # Return the max time if not solved   

def RRT3D(start, subgoal, zone_start, zone_next, obstacles, env):
    space = ob.RealVectorStateSpace(3)
    
    lower_bound = [min(zone_start[0], zone_next[0]), min(zone_start[1], zone_next[1]), min(zone_start[2], zone_next[2])]
    upper_bound = [max(zone_start[3], zone_next[3]), max(zone_start[4], zone_next[4]), max(zone_start[5], zone_next[5])]
    
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(0, lower_bound[0])
    bounds.setLow(1, lower_bound[1])
    bounds.setLow(2, lower_bound[2])
    bounds.setHigh(0, upper_bound[0])
    bounds.setHigh(1, upper_bound[1])
    bounds.setHigh(2, upper_bound[2])
    
    space.setBounds(bounds)
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(lambda state: is_state_valid_box(state, env)))
    
    start_state = ob.State(space)
    start_state()[0] = start[0]
    start_state()[1] = start[1]
    start_state()[2] = start[2]
    
    goal_state = ob.State(space)
    goal_state()[0] = subgoal[0]
    goal_state()[1] = subgoal[1]
    goal_state()[2] = subgoal[2]
    
    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start_state, goal_state)
    
    planner = og.RRT(si)
    planner.setRange(3) 
    planner.setProblemDefinition(pdef)
    planner.setup()
    startRRT=time.time()
    print("--------------")
    solved = planner.solve(1.5)  # Allow 10 seconds to solve
    endRRT=time.time()
    iteration_count = 1
    print("endRRT-startRRT",endRRT-startRRT)
    if endRRT- startRRT <1.5:
        path = pdef.getSolutionPath()
        new_path = [(state[0], state[1], state[2]) for state in path.getStates()]
        planning_time = endRRT-startRRT  # Get the planning time
        return planning_time, new_path, 1,iteration_count
    else:
        return 100, [], 0,iteration_count