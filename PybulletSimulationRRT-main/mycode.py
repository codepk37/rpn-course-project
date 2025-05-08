import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import myclass as panda_sim
import cv2
#video requires ffmpeg available in path
createVideo=False
fps=120.
timeStep = 1./fps

if createVideo:
	p.connect(p.GUI, options="--minGraphicsUpdateTimeMs=0 --mp4=\"pybullet_grasp.mp4\" --mp4fps="+str(fps) )
else:
	p.connect(p.GUI)



# p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
p.resetDebugVisualizerCamera(cameraDistance=.4, cameraYaw=-170, cameraPitch=200, cameraTargetPosition=[.0, -.0, 0.])
p.setAdditionalSearchPath(pd.getDataPath())

p.setTimeStep(timeStep)
p.setGravity(0,0,-9.8)

panda = panda_sim.PandaSimAuto(p,[0,0,0])
panda.control_dt = timeStep

logId = panda.bullet_client.startStateLogging(panda.bullet_client.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
panda.bullet_client.submitProfileTiming("start")
for i in range (100000):
	panda.bullet_client.submitProfileTiming("full_step")
	panda.step()
	p.stepSimulation()



	if createVideo:
		p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
	if not createVideo:
		time.sleep(timeStep)
	panda.bullet_client.submitProfileTiming()
panda.bullet_client.submitProfileTiming()
panda.bullet_client.stopStateLogging(logId)



# # Define the transformation matrices ; in world frame
# princle_grasp = np.array([
#     [-0.4132756,   0.18805644,  0.8909759,   0.0992611],
#     [-0.90863952, -0.14943159, -0.38992862, -0.65431479],
#     [-0.05981135,  0.97072385, -0.23263197,  0.15277411],
#     [0.0,          0.0,         0.0,         1.0]
# ])

# box_grasp = np.array([
#     [0.9967264,  -0.08043922, -0.00812833,  0.01989224],
#     [0.06884508,  0.89715319, -0.43632161, -0.52965904],
#     [-0.04238977, -0.43433366, -0.89975406,  0.24607036],
#     [0.0,         0.0,         0.0,         1.0]
# ])

# mustard_grasp = np.array([
#     [0.98065335, -0.11697425, -0.1569589,   0.1821094],
#     [0.11360994,  0.99306421, -0.03026898, -0.55333701],
#     [-0.159411,  -0.01185129, -0.98714108,  0.14191832],
#     [0.0,         0.0,         0.0,         1.0]
# ])