import time
import numpy as np
import math
from allfunc import *

import pybullet as p
camera_position = np.array([0.1, -0.3, .7])
camera_target = np.array([0.1, -0.45,0.0])
camera_up_vector = np.array([0, 1,0])
view_matrix = p.computeViewMatrix(cameraEyePosition=camera_position,
                                  cameraTargetPosition=camera_target,
                                  cameraUpVector=camera_up_vector)
view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F") #(4, 4)

print(view_matrix)

def getcameratoworldpos_o(Graspincamera):
    
    # Graspincamera[:, 2, :] *= -1

    world_grasp = np.linalg.inv(view_matrix)  @ Graspincamera 
    translation= world_grasp[:3, 3]
    # print("trans1 ",translation)
    rotation_matrix = world_grasp[:3,:3]

    rotation_matrix2 =np.array([ #because panda arm is 90 deg rotated comapred to arm in constact graspnet
        [0,1,0],
        [1,0,0],
        [0,0,1]
    ])

    quaternion = rotation_matrix_to_quaternions( rotation_matrix@ rotation_matrix2)

    translation+=world_grasp[:3, 2]*0.9 #translate along grasp direction, according to arm+ consideration of backwardness as in contact graspnet
    # print("translation ",translation)
    return (translation),quaternion



def getcameratoworldpos_o_pre(Graspincamera):#pregrasp
    
    # Graspincamera[:, 2, :] *= -1

    world_grasp = np.linalg.inv(view_matrix)  @ Graspincamera 
    translation= world_grasp[:3, 3]
    # print("trans1 ",translation)
    rotation_matrix = world_grasp[:3,:3]

    rotation_matrix2 =np.array([ #because panda arm is 90 deg rotated comapred to arm in constact graspnet
        [0,1,0],
        [1,0,0],
        [0,0,1]
    ])

    quaternion = rotation_matrix_to_quaternions( rotation_matrix@ rotation_matrix2)

    translation-=world_grasp[:3, 2]*0.2 #translate along grasp direction, according to arm+ consideration of backwardness as in contact graspnet
    print("translation pre",translation,quaternion)
    return (translation),quaternion


useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
jointPositions=[-.98, -0.458, 0.31, -2.24, -0.30, 2.66, 0.32, 0.02, 0.02]
rp = jointPositions

from slerp import *
from lerp import *
import sys
# Check for arguments
if len(sys.argv) != 3:
    print("Usage: python creating_pipeline.py <urdf_file> <grasp_file>")
    sys.exit(1)

urdf_file = sys.argv[1]
grasp_file = sys.argv[2]

class PandaSim(object):
  def __init__(self, bullet_client, offset):
    self.bullet_client = bullet_client
    self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
    self.offset = np.array(offset)
    
    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    self.legos=[]
    
    self.bullet_client.loadURDF("table/table.urdf", [0+offset[0], -0.6+offset[1]-0.026, -0.6+offset[2]], [0,0,0,1], flags=flags)
    #   # self.bullet_client.loadURDF("tray/traybox.urdf", [0+offset[0], 0+offset[1], -0.6+offset[2]], [-0.5, -0.5, -0.5, 0.5], flags=flags)
    #   # self.bullet_client.loadURDF("kitchen.urdf", [0+offset[0], 0+offset[1], -0.6+offset[2]], [-0.5, -0.5, -0.5, 0.5], flags=flags)

    # self.legos.append(self.bullet_client.loadURDF("/home/pavan/Desktop/my_pybullet-main/Other_urdf/003_cracker_box.urdf",np.array([0.1, -0.3, 0.5])+self.offset, flags=flags,globalScaling=0.08))
    # self.chips=self.bullet_client.loadURDF("YcbChipsCan/model.urdf",np.array([0.2,  -0.7,0.1])+self.offset, flags=flags)
    # # self.sphereId = self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0.3,0,0.])+self.offset, flags=flags)
    # self.cracker =self.bullet_client.loadURDF("YcbCrackerBox/model.urdf",np.array( [0, -0.6, 0.1])+self.offset, flags=flags)
    # self.mustard_bottle_id = self.bullet_client.loadURDF("YcbMustardBottle/model.urdf",np.array( [0.1, -0.5, 0.3])+self.offset,[1,0,0,1], flags=flags)
    object_urdf_path = urdf_file  # The URDF file passed as an argument
    self.objectflag = flags
    # Check if any of the specified numbers are in the file name
    # if any(num in urdf_file for num in ("005", "010","012",'013','015','016','017','018',"036"  ,'055' ,"022", "040", "044")):#coz they roll
    self.objectflag = self.bullet_client.URDF_USE_INERTIA_FROM_FILE

    self.curobj = bullet_client.loadURDF(object_urdf_path, np.array([0.1, -0.4, 0.2]), flags=self.objectflag, globalScaling=0.08)

    grasp_file
    data = np.load(grasp_file, allow_pickle=True)
    top_30_grasps = data['top_30_grasps_dict'].item()
    self.top_30_grasps= top_30_grasps[255][::-1]  #reversed ->now first grasp has highest score
    self.top_30_grasps[:, 2, :] *= -1
    # self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, -0., 0.])+self.offset, flags=flags)
    orn= [0,0,0,1] #[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
    # eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
    self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
    
    self.graspid =0 #upto 7
    self.stability_threshold_rotation = 0.1  # Define your rotation threshold (e.g., radians)
    self.stability_threshold_translation = 0.01  # Define translation threshold (e.g., meters)

    index = 0
    self.state = 0
    self.control_dt = 1./240.
    self.finger_target = 0
    self.gripper_height = 0.2
    self.pose = (-0.1,-0.2,0.45)
    self.o= (0.66152662,  0.71572312, -0.16721249 , 0.14887228)
    self.globalpose= (-0.1,-0.2,0.45)
    self.globalo   = (0.66152662,  0.71572312, -0.16721249 , 0.14887228)
    self.iter_t=0
    #create a constraint to keep the fingers centered
    c = self.bullet_client.createConstraint(self.panda,
                       9,
                       self.panda,
                       10,
                       jointType=self.bullet_client.JOINT_GEAR,
                       jointAxis=[1, 0, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
    self.bullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
 
    for j in range(self.bullet_client.getNumJoints(self.panda)):
      self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.panda, j)
      #print("info=",info)
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
    self.t = 0.


    # Get the number of joints in the arm
    num_joints = p.getNumJoints(self.panda)

    # Disable collision for each link (except the base)
    # for joint_index in range(1, num_joints):  # Skip the base link
    #     # Set collision group and mask to 0 to disable collision
    #     p.setCollisionFilterGroupMask(self.panda, joint_index, 0, 0)  # Disables collision for that link


  def reset(self):
    pass

  def update_state(self):
    keys = self.bullet_client.getKeyboardEvents()
    if len(keys)>0:
      for k,v in keys.items():
        if v&self.bullet_client.KEY_WAS_TRIGGERED:
          if (k==ord('1')):
            self.state = 1
          if (k==ord('2')):
            self.state = 2
          if (k==ord('3')):
            self.state = 3
          if (k==ord('4')):
            self.state = 4
          if (k==ord('5')):
                self.state = 5
          if (k==ord('6')):
                self.state = 6
        if v&self.bullet_client.KEY_WAS_RELEASED:
            self.state = 0
  def step(self):
    if self.state==6:
      self.finger_target = 0.0001
    if self.state==5:
      self.finger_target = 1.1 
    self.bullet_client.submitProfileTiming("step")
    self.update_state()
    #print("self.state=",self.state)
    # print("self.finger_target=",self.finger_target)

    # self.gripper_height = alpha * self.gripper_height + (1.-alpha)*0.03 # when goes to grasp position -4 state
    if  self.state == 3 : #initialize position
      # self.gripper_height =  0.6  #other time ,higer heigh -all other state
      self.globalpose
      self.globalo
      self.finalpose = (-0.1,-0.2,0.5)
      self.finalo= (0.66152662,  0.71572312, -0.16721249 , 0.14887228)
      if self.iter_t<1:
        self.iter_t+=0.01
        # self.pose,self.o =  
        self.pose = lerp(self.globalpose,self.finalpose,self.iter_t)
        # self.o = slerp(self.globalo,self.finalo,self.iter_t)

    if  self.state == 4 : #initialize position
      # self.gripper_height =  0.6  #other time ,higer heigh -all other state
      self.globalpose
      self.globalo
      self.finalpose = (-0.1,-0.2,0.6)
      self.finalo= (1,0,0,0)
      if self.iter_t<1:
        self.iter_t+=0.01
        # self.pose,self.o =  
        # self.pose = lerp(self.globalpose,self.finalpose,self.iter_t)
        self.o = slerp(self.globalo,self.finalo,self.iter_t)

      
    # print(pos,"p")
    gripper_pos, gripper_orn = self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex)[:2]
    # print(gripper_pos,"g")
    
    if self.state == 11 :
      self.globalpose
      self.globalo
      self.finalpose , self.finalo = getcameratoworldpos_o_pre(self.top_30_grasps[self.graspid]) #grasp3_set3 
      
      if self.iter_t<1:
        self.iter_t+=0.009
        # self.pose,self.o =  
        self.pose = lerp(self.globalpose,self.finalpose,self.iter_t)
        # self.o = slerp(self.globalo,self.finalo,self.iter_t)


    
    if self.state == 12 :
      # print("state 12")

      self.finalpose , self.finalo = getcameratoworldpos_o(self.top_30_grasps[self.graspid]) #grasp3_set3 
      
      if self.iter_t<1:
        self.iter_t+=0.01
        # self.pose,self.o =  
        self.pose = lerp(self.globalpose,self.finalpose,self.iter_t)
        # self.o = slerp(self.globalo,self.finalo,self.iter_t)

 

    if self.state == 13 :
      # print("state 12")

      self.globalpose
      self.globalo
      self.finalpose , self.finalo = getcameratoworldpos_o(self.top_30_grasps[self.graspid]) #grasp3_set3 
      
      if self.iter_t<1:
        self.iter_t+=0.1
        # self.pose,self.o =  
        # self.pose = lerp(self.globalpose,self.finalpose,self.iter_t)
        self.o = slerp(self.globalo,self.finalo,self.iter_t)


    if self.state == 7: #go outside

      self.finalpose =(-0.30, -0.3, 0.50)
      self.globalpose
      if self.iter_t<1:
        self.iter_t+=0.009
        self.pose = lerp(self.globalpose,self.finalpose,self.iter_t)
      

    if self.state == 8: #drop
      self.finalpose =(-0.30, -0.3, 0.30)
      self.globalpose
      if self.iter_t<1:
        self.iter_t+=0.01
        self.pose = lerp(self.globalpose,self.finalpose,self.iter_t)


    if self.state==99: #pause state
        self.pose= self.pose
        self.o= self.o

    # orn = self.bullet_client.getQuaternionFromEuler([100,0,0])
    self.bullet_client.submitProfileTiming("IK")
    jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, self.pose, self.o, ll, ul,
      jr, rp, maxNumIterations=20)
    self.bullet_client.submitProfileTiming()
    for i in range(pandaNumDofs):
      self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
    #target for fingers
    for i in [9,10]:
      self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target ,force= 1000)
    self.bullet_client.submitProfileTiming()


  def get_relative_pose(self, object_id):
    """Update the pose of the object relative to the gripper."""
    gripper_pos, gripper_orn = self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex)[:2]
    object_pos, object_orn = self.bullet_client.getBasePositionAndOrientation(object_id)

    # Compute relative transformation (object pose relative to the gripper)
    relative_pos = np.array(object_pos) - np.array(gripper_pos)
    # relative_orn = p.getDifferenceQuaternion(object_orn, gripper_orn)
    # angle_change = 2 * np.arccos(abs(relative_orn[3]))  # Quaternion to angle change
    # return relative_pos, angle_change
    object_orn = p.getMatrixFromQuaternion(object_orn)
    return relative_pos, object_orn



class PandaSimAuto(PandaSim):
  def __init__(self, bullet_client, offset):
    PandaSim.__init__(self, bullet_client, offset)
    self.state_t = 0
    self.cur_state = 0

    self.states=[ 3,4,  5,  13, 11,12, 6,7,8, 99,5   ] # ,   3,5,12,6,7,8,5,      3,5,13,6,7,8,5
    self.state_durations=[1,2, 2,2,2.2,2.2,2,3,2,1,2  ] # ,  .5,1,1,1,2,2,1   , .5,1,1,1,2,2,1


  def update_state(self):
    self.state_t += self.control_dt
    if self.state_t > self.state_durations[self.cur_state]:
      self.cur_state += 1
      if self.cur_state>=len(self.states):
        self.cur_state = 0
        self.graspid= (self.graspid+1)#%30
        
        if self.graspid >15 :
            filename = grasp_file.split("\\")[-1].split(".")[0]
            message = f"{filename} Unsuccessful till Grasp 15\n"
            print(message)
            with open("results_noted.txt", "a") as file:
                file.write(message)
            exit(0)
        
        self.bullet_client.removeBody(self.curobj)
        # self.mustard_bottle_id = self.bullet_client.loadURDF("YcbMustardBottle/model.urdf",np.array( [0.1, -0.5, 0.3])+self.offset,[1,0,0,1], flags=flags)
        object_urdf_path = urdf_file  # The URDF file passed as an argument path
        self.curobj = self.bullet_client.loadURDF(object_urdf_path, np.array([0.1, -0.5, 0.2]), flags=self.objectflag, globalScaling=0.08)

        print("graspID",self.graspid)
      # print("curstate",self.states[self.cur_state])
      self.state_t = 0
      self.state=self.states[self.cur_state]

      self.globalpose,self.globalo = self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex)[:2]
      # print("globalo ", self.globalpose,"    ", self.globalo)
      self.iter_t=0

      # print("self.state=",self.state)
      if self.state==7: #just before picking grasped
        self.rel_pose1,self.object_orn1= self.get_relative_pose(self.curobj)
        # print("state 7",self.rel_pose1,np.linalg.norm(self.rel_pose1))


      if self.state==6:
         fingers_joint_indices = [9, 10]  # Usually, the last two joints of the Panda arm
         for joint_index in fingers_joint_indices:
            p.setCollisionFilterGroupMask(self.panda, joint_index, 1, 1)  # Re-enable collision for fingers
      if self.state==5:
         fingers_joint_indices = [9, 10]  # Usually, the last two joints of the Panda arm
         for joint_index in fingers_joint_indices:
            p.setCollisionFilterGroupMask(self.panda, joint_index, 0, 0) 



      if self.state==99:#just before opening grasp to drop
        rel_pose2,self.object_orn2= self.get_relative_pose(self.curobj)
        # print("state 99",rel_pose2,np.linalg.norm(rel_pose2))
        self.object_orn1= np.array(self.object_orn1).reshape(3, 3)
        self.object_orn2= np.array(self.object_orn2).reshape(3, 3)
        rotation_diff = self.object_orn1 - self.object_orn2
        frobenius_norm = np.sqrt(np.sum(rotation_diff ** 2))
        print("In World Frame, Frobenius norm of at_grasping,before_dropping ",frobenius_norm)
        dis=abs(np.linalg.norm(self.rel_pose1)- np.linalg.norm(rel_pose2))
        print("distsance slip ",dis)
        if dis<0.1:
          
          filename = grasp_file.split("\\")[-1].split(".")[0]
          message = f"{filename} Successfully Lifted at Grasp {self.graspid} ,  dis ={dis}\n"
          print(message)
          with open("results_noted.txt", "a") as file:
              file.write(message)

          exit(0)