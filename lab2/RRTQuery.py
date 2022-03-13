import time
import pickle
import numpy as np

import RobotUtil as rt
from franka_robot import FrankaRobot 
from collision_boxes_publisher import CollisionBoxesPublisher
import Locobot
import rospy
import time
from frankapy import FrankaArm


def FindNearest(prevPoints,newPoint):
	D=np.array([np.linalg.norm(np.array(point)-np.array(newPoint)) for point in prevPoints])
	return D.argmin()

fr = FrankaRobot()

fa = FrankaArm()

np.random.seed(0)
deg_to_rad = np.pi/180.

#Initialize robot object
mybot=Locobot.Locobot()

pointsObs=[]
axesObs=[]


########  TODO: Fill in Box Parameters Here  ############################################
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.], [0, 0, 0]),[0, 0, 0])
pointsObs.append(envpoints), axesObs.append(envaxes)

#########################################################################################

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.], [0.15, 0.46, 0.5]),[1.2, 0.01, 1.1])
pointsObs.append(envpoints), axesObs.append(envaxes)
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.], [0.15, -0.46, 0.5]),[1.2, 0.01, 1.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.], [-0.41, 0, 0.5]),[0.01, 1, 1.11])
pointsObs.append(envpoints), axesObs.append(envaxes)
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.], [0.75, 0, 0.5]),[0.01, 1, 1.1])
pointsObs.append(envpoints), axesObs.append(envaxes)
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.], [0.2, 0, 1]),[1.2, 1, 0.01])
pointsObs.append(envpoints), axesObs.append(envaxes)
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.], [0.2, 0, -0.05]),[ 1.2, 1, 0.01])
pointsObs.append(envpoints), axesObs.append(envaxes)


############# TODO: Define Start and Goal Joints #######################
qGoal=None
qInit=None

#TODO - Create RRT to find path to a goal configuration
rrtVertices=[]
rrtEdges=[]

rrtVertices.append(qInit)
rrtEdges.append(0)
thresh = 0.25
FoundSolution = False

while len(rrtVertices) < 3000 and not FoundSolution:
	print(len(rrtVertices))

### if a solution was found

if FoundSolution:
	# Extract path
	plan = []
	c = -1  # Assume last added vertex is at goal
	plan.insert(0, rrtVertices[c])

	while True:
		c = rrtEdges[c]
		plan.insert(0, rrtVertices[c])
		if c == 0:
			break

	# TODO - Path shortening
	for i in range(150):
		pass

	robot = vpi.vBot()
	robot.connect()

	for q in plan:
		robot.move(q)
		time.sleep(1)

	robot.destroy()

else:
	print("No solution found")
