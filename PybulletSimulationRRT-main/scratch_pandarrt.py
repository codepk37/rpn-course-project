from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import random
import time
import argparse


UR5_JOINT_INDICES = [0, 1, 2,3,4,5,6]



def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        # p.resetJointState(body, joint, value)
        p.setJointMotorControl2(body, joint, p.POSITION_CONTROL, value,force=5 * 240.)

def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def remove_marker(marker_id):
   p.removeBody(marker_id)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--smoothing', action='store_true', default=False)
    args = parser.parse_args()
    return args

def find_nn(V, q):
    """Find the nearest node from q in V.
    
    Arguments:
        V {list of numpy arrays} -- The list of nodes in the RRT {[,,],[]} config stored 
        q {numpy array} -- a configuration
    
    Returns:
        numpy array -- q_near: the nearest node in V from q
        int -- The index of q_near in V
    """
    distances = np.linalg.norm(V - q, axis=1)

    nearest_idx = np.argmin(distances)

    return V[nearest_idx], nearest_idx

def progress_by_step_size(q_src, q_dest  ,q_goal):
    """Progress from q_src to q_dest by step_size
    
    Arguments:
        q_src {numpy array} -- The start configuration
        q_dest {numpy array} -- The end configuration
    
    Returns:
        numpy array -- q_new: a node step_size away from q_src
            heading in the direction of q_dest
    """

    diff = q_dest - q_src
    norm = np.linalg.norm(diff)
    delta = (step_size / norm) * diff

    ret = q_src + delta

    if np.linalg.norm(q_goal - ret) < step_size: #/2 step size
        delta = ((step_size / norm) * diff )/2
        ret = q_src + delta

    return ret

def build_path(start_idx, goal_idx, V, E):
    """Build the graph and find a path from start to goal index in T(V, E).
    
    Arguments:
        start_idx {int} -- index of q_start in V
        goal_idx {int} -- index of q_goal in V
        V {list of numpy arrays} -- The list of nodes in the RRT
        E {list of index pairs} -- The list of edges in the RRT
    
    Returns:
        {list of numpy arrays} -- The list of nodes that form a path 
            from q_start to q_goal in RRT. Return None if path between start 
            and goal index is not found. 
    """

    # build a graph
    adj_list = { #creates an adjacency list (adj_list) for the graph
        k:[]        
        for k in range(len(V))
    }
    for edge in E:
        adj_list[edge[0]].append(edge[1])
        adj_list[edge[1]].append(edge[0])

    path_idx = find_path(start_idx, goal_idx, adj_list, 
        visited=set(), final_path=[])
    
    if path_idx is not None:
        return path_idx

def find_path(start_idx, goal_idx, adj_list, visited=set(), final_path=[]):
    """Find a path between start and goal based on adjacency list with DFS.
    
    Arguments:
        start_idx {int} -- index of q_start in V
        goal_idx {int} -- index of q_goal in V
        adj_list {dict[int]: list[int]} -- the adjacency list that map node index to
            its neighbouring node indexes
    
    Keyword Arguments:
        visited {set} -- Set of visited node indexes (default: {set()})
        final_path {list} -- The path from start node index to current node index 
            (default: {[]})
    
    Returns:
        list of int -- The list of node indexes that forms a path from q_start and 
            q_goal. Return None if path between start and goal index is not found.
    """
    visited.add(start_idx)
    final_path.append(start_idx)

    if start_idx == goal_idx:
        return final_path

    for n in adj_list[start_idx]: #uses DFS
        if n not in visited:
            res = find_path(n, goal_idx, adj_list, 
                visited, final_path)

            if res is not None:
                return res
    
    # backtrack try different node
    final_path.pop()
    visited.remove(start_idx)
    
def extend_rrt(V,V_J, E, q_rand, world_pos, color ,q_goal):
    """Extends the RRT by finding the nearest node on the tree for a given 
    random point q_rand.
    
    Arguments:
        V {list of numpy arrays} -- The list of nodes in the RRT
        E {list of index pairs} -- The list of edges in the RRT
        q_rand {numpy array} -- a random configuration
        world_pos {list of list of floats} -- The list of world positions
        color {list of floats} -- The RGB color for drawing the path [R, G, B]
    
    Returns:
        numpy array -- a new configuration collision-free between q_near and q_rand.
            Return None if the configuration is not collision-free.
    """
    
    
    q_near, q_near_idx = find_nn(V, q_rand) #finds nearest node in Tree V
    
    # avoid division by 0
    if np.linalg.norm(q_rand - q_near) == 0.0:
        return

    # move from q_near to q
    q_new = progress_by_step_size(q_near, q_rand  ,q_goal) #move by step size towards random node,get Tree Vertice
    # find jointangle for this pos,o
    # print("hi ",q_new[:3],q_new[3:])
    joint_states = [p.getJointState(ur5, i)[0] for i in range(p.getNumJoints(ur5))]
    # print("Current joint angles:", joint_states)
    jointPoses = p.calculateInverseKinematics(ur5,pandaEndEffectorIndex, q_new[:3], q_new[3:], ll, ul,
      jr, joint_states, maxNumIterations=200)
    q_new_joint = jointPoses[:7]


    # print("q new  ",collision_fn(q_new_joint))
    if not collision_fn(q_new_joint): #func:i think can be generalized to franka panda
        cur_world_pos,_ = p.getLinkState(ur5,11)[:2] #gets current world pos,o 
        V.append(q_new)
        V_J.append(q_new_joint)

        # insert index of node to E
        E.append((q_near_idx, len(V)-1)) #append nearest verdex id , my new id
            # [0,1],[1,2],[2,3],[1,4]

        world_pos.append(cur_world_pos) #add link's world pos, will be id wise initutively
        
        p.addUserDebugLine(
            lineFromXYZ=world_pos[q_near_idx], #gotten nearest node id's world
            lineToXYZ=cur_world_pos,  # current link's world pos
            lineColorRGB=color, 
            lineWidth = 0.5
        )
        # print("q  ",q_new)
        return q_new

def rrt():
    """Implements a RRT algorithm. 
    
    1. Select a random configuration q_rand (q_rand = q_goal with certain bias prob)
    2. Extend the RRT towards q_rand by q_new, a node step-size away from q_near in RRT.
    3. If q_new is within step_size away from q_goal, goal is found and build a path 
        from q_start to q_goal.
    
    Expect:
        step_size [float]: step size for growing the tree
        bias [float]: the bias probability towards q_goal
        lower_bound [float]: the lower boundary value of a possible random configuration
        upper_bound [float]: the upper boundary value of a possible random configuration
        q_start [numpy array]: the start configuration
        q_goal [numpy array]: the goal configuration
        start_position [list of floats]: the start position of the world coordinate

    Returns:
        list of numpy array -- the path from q_start to q_goal if exist. 
            Return None if path is not found
    """
    V = [q_start] # insert q_start ,[x,y,z, x,y,z,w]
    V_J= [startjoints]  #jointangle for it 
    E = []
    world_pos = [start_position]

    ###############################################
    # TODO your code to implement the rrt algorithm
    ###############################################
    
    is_goal_found = False
    for idx in range(N):
        follow_goal = np.random.choice([False, True], p=[1-bias, bias])
        if follow_goal:
            q_rand = q_goal
        else:
            q_rand = np.random.uniform(lower_bound, upper_bound)
        
        q_new = extend_rrt(V,V_J, E, q_rand, world_pos, [0, 1, 0],  q_goal)

        print(q_new)

        if q_new is not None and np.linalg.norm(q_goal - q_new) < step_size/2:
            print("goal is found!",np.linalg.norm(q_goal - q_new) )
            is_goal_found = True
            break
    
    if is_goal_found:
        print("path building....")
        path_idx = build_path(0, len(V_J) - 1, V_J, E) #[0,1,2,3,5,7,9,14]
        path = [V_J[idx] for idx in path_idx]
        print("path building done!")
        
        return path





if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=-40.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('franka_panda/panda.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    c = p.createConstraint(ur5,
                       9,
                       ur5,
                       10,
                       jointType=p.JOINT_GEAR,
                       jointAxis=[1, 0, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
    p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
    index = 0
    rp=[-.98, -0.458, 0.31, -2.24, -0.30, 2.66, 0.32, 0.02, 0.02,0.4000000178970985, 0.40000000000000015, 0.0] #descibes current state, will give ik joints according near to it, updating it is good
    for j in range(p.getNumJoints(ur5)):
      p.changeDynamics(ur5, j, linearDamping=0, angularDamping=0)
      info = p.getJointInfo(ur5, j)
      #print("info=",info)
      jointName = info[1]
      jointType = info[2]
      if (jointType == p.JOINT_PRISMATIC):
        
        p.resetJointState(ur5, j, rp[index]) 
        index=index+1
      if (jointType == p.JOINT_REVOLUTE):
        p.resetJointState(ur5, j, rp[index]) 
        index=index+1

    curobj = p.loadURDF("./Other_urdf/004_sugar_box.urdf", np.array([0.1, -0.5, .1]), flags=p.URDF_USE_INERTIA_FROM_FILE, globalScaling=0.08)
    obstacles =[curobj]#,plane obstacle1, obstacle2]

    # start and goal
    start_position= (.4,-.6,.4)
    startxyzo=[-0.1,-0.2,0.5 ,0.66152662,  0.71572312, -0.16721249 , 0.14887228] #later change  findnn,onwards for quaternion consideration. for now same quaternion
    
    # startxyzo =[ 0.10230514 ,-0.21649526  ,0.32916625 ,-0.60604327,  0.61379097 ,-0.34115129 ,-0.37361474]
    # goalxyzo=[ 0.09379795, -0.47267813,  0.12446251 ,-0.12056493 , 0.30668818 , 0.68324747 , 0.57995989]
    goalxyzo=[  -0.4463171 ,-0.36467484 , 0.54619679 ,-0.60604327,  0.61379097 ,-0.34115129 ,-0.37361474]
    pandaEndEffectorIndex = 11
    pandaNumDofs = 7
    ll = [-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671]
    ul = [2.9671, 1.8326, 2.9671, 0.0, 2.9671, 3.8223, 2.9671]
    jr = [ul[i] - ll[i] for i in range(pandaNumDofs)]
        #restposes for null space

    jointPoses = p.calculateInverseKinematics(ur5,pandaEndEffectorIndex, startxyzo[:3], startxyzo[3:], ll, ul,
      jr, rp, maxNumIterations=20)
    print(jointPoses)
    startjoints = jointPoses[:7]
    ##
    jointPoses1 = p.calculateInverseKinematics(ur5,pandaEndEffectorIndex, goalxyzo[:3], goalxyzo[3:], ll, ul,
      jr, rp, maxNumIterations=20)
    # print(jointPoses1)
    endjoints = jointPoses1[:7]
    ##
    goal_position = (0.5317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goalxyzo[:3], radius=0.02, color=[0, 1, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, startjoints)#start_conf)    

    # define constants
    N = 1000000
    smooth_count = 100
    step_size = 0.04#0.09
    bias = 0.5#0.5
    
    lower_bound = np.array([-0.4, -0.8, -0.05, -1, -1, -1, -1])
    upper_bound = np.array([0.4, 0.05, 0.6, 1, 1, 1, 1])

    q_start = np.array(startxyzo)
    q_goal = np.array(goalxyzo)

    # place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn, get_joint_positions
    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                       attachments=[], self_collisions=False,
                                       disabled_collisions=set())

    from save_load import *
    path_conf = load_path("004_sugar_b",id=1)
    # If no saved path exists, compute new one
    if path_conf is None:
        # using rrt
        path_conf = rrt()
        if path_conf is not None:
            save_path(path_conf, "004_sugar_box",id=1)

    print("path_conf ",path_conf)
    print(path_conf[-1])

    if path_conf is None:
        # pause here
        save_path(path_conf, "003_cracker_box")
        input("no collision-free path is found within the time budget, finish?")
    else:
        ###############################################
        # TODO your code to highlight the solution path
        ###############################################

        # execute the path
        draw_path = False
        while True:
            # print("get joint pos ",get_joint_positions(ur5,UR5_JOINT_INDICES)) #gives joint angles ,same as last q value
            if not draw_path:
                for q in path_conf:
                    set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                    cur_world_pos = p.getLinkState(ur5, 11)[0] #link number ,[0] pose ,x,y,z ; [1] quaternion x,y,z,w
                    # print("cur ",cur_world_pos)
                    draw_sphere_marker(cur_world_pos, 0.02, 
                        [1, 0, 0, 1])

                draw_path = True
            
            for q in path_conf:
                # print("path conf ",q) [,,]
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                gripper_pos, gripper_orn = p.getLinkState(ur5, pandaEndEffectorIndex)[:2]
                print("pose , or ",gripper_pos , gripper_orn)
                time.sleep(.3)
            exit(0)
