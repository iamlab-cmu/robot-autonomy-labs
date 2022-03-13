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
# else:
# rospy.init_node('rrt')
np.random.seed(4)
# deg_to_rad = np.pi/180.

#Initialize robot object
mybot=Locobot.Locobot()

#Create environment obstacles - # these are blocks in the environment/scene (not part of robot) 
boxes = np.array([
	# obstacle
	[0.44680895, -0.00402793, 0.126119755, 0, 0, 0, 0.17,0.1,0.26],
	# sides
# 	[0.15, 0.46, 0.5, 0, 0, 0, 1.2, 0.01, 1.1],
# 	[0.15, -0.46, 0.5, 0, 0, 0, 1.2, 0.01, 1.1],
# 	# back
# 	[-0.41, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
# 	# front
# 	[0.75, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
# 	# top
# 	[0.2, 0, 1, 0, 0, 0, 1.2, 1, 0.01],
# 	# bottom
# 	[0.2, 0, -0.05, 0, 0, 0, 1.2, 1, 0.01]
])
pointsObs=[]
axesObs=[]

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.], [0.44680895, -0.00402793, 0.126119755]),[0.33,0.1,0.3])
pointsObs.append(envpoints), axesObs.append(envaxes)

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


# define start and goal
deg_to_rad = np.pi/180.
qGoal=[-0.53382925,  0.10712698,  0.01386268, -2.45663784,  0.05328225,
		2.64689502,  0.22742127] 
qInit=[-0.28568509,  0.08583521,  0.83456529, -2.4694526 ,  0.09138187,
		2.59050709,  1.2237876 ] 
# mybot.PlotCollisionBlockPoints(qInit, pointsObs)
# mybot.PlotCollisionBlockPoints(qGoal, pointsObs)

#TODO - Create RRT to find path to a goal configuration
rrtVertices=[]
rrtEdges=[]

rrtVertices.append(qInit)
rrtEdges.append(0)

# thresh=0.25
thresh=0.1
# thresh=0.01
FoundSolution=False

while len(rrtVertices) < 1000 and not FoundSolution:
	print(len(rrtVertices))
	qRand = mybot.SampleRobotConfig()
	if np.random.uniform(0, 1) < 0.01:
		qRand = qGoal
	idNear = FindNearest(rrtVertices, qRand)
	qNear = rrtVertices[idNear]
	while np.linalg.norm(np.array(qRand) - np.array(qNear)) > thresh:
		subtracted_arr = np.array(qRand) - np.array(qNear)

		qConnect = np.array(qNear) + (thresh * subtracted_arr)/np.linalg.norm(subtracted_arr)
		if not mybot.DetectCollisionEdge(qConnect, qNear, pointsObs, axesObs):
			rrtVertices.append(qConnect)
			rrtEdges.append(idNear)
			qNear = qConnect
		else: 
			break
	# if np.linalg.norm(subtracted_arr) > thresh:
	# 	qConnect = np.array(qNear) + (thresh * subtracted_arr)/np.linalg.norm(subtracted_arr)
	# else:
	qConnect = qRand
	if not mybot.DetectCollisionEdge(qConnect, qNear, pointsObs, axesObs):
		rrtVertices.append(qConnect)
		rrtEdges.append(idNear)
	idNear = FindNearest(rrtVertices, qGoal)
	if np.linalg.norm(np.array(qGoal)-np.array(rrtVertices[idNear])) < 0.025:
		rrtVertices.append(qGoal)
		rrtEdges.append(idNear)
		FoundSolution = True
		break


### if a solution was found
		
if FoundSolution:
	# Extract path
	plan=[]
	c=-1 #Assume last added vertex is at goal 
	plan.insert(0, rrtVertices[c])

	while True:
		c=rrtEdges[c]
		plan.insert(0, rrtVertices[c])
		if c==0:
			break

	# TODO - Path shortening
	# for i in range(150):
		# # pass
		# anchorA = np.random.randint(0, len(plan) - 2)
		# anchorB = np.random.randint(anchorA + 1, len(plan) - 1)
		# shiftA = np.random.uniform(0,1)
		# shiftB = np.random.uniform(0,1)
		# candidateA = (1 - shiftA)* np.array(plan[anchorA]) + shiftA*np.array(plan[anchorA + 1])
		# candidateB = (1 - shiftB)* np.array(plan[anchorB]) + shiftB*np.array(plan[anchorB + 1])
		# if not mybot.DetectCollisionEdge(candidateA, candidateB, pointsObs, axesObs):
		# 	while anchorB > anchorA:
		# 		plan.pop(anchorB)
		# 		anchorB = anchorB - 1
		# 	plan.insert(anchorA + 1, candidateB)
		# 	plan.insert(anchorA + 1, candidateA)
	fa.reset_joints()
	for joint in plan:
		fa.goto_joints(joint)
	# collision_boxes_publisher = CollisionBoxesPublisher('collision_boxes')
	# rate = rospy.Rate(1)
	# i = 0
	# print(plan)
	# while not rospy.is_shutdown():
	# 	rate.sleep()
	# 	time.sleep(2)
	# 	joints = plan[i % len(plan)]
	# 	mybot.PlotCollisionBlockPoints(joints, pointsObs)

	# 	fr.publish_joints(joints)
	# 	fr.publish_collision_boxes(joints)
	# 	collision_boxes_publisher.publish_boxes(boxes)
	# 	i +=1
		
	# robot=vpi.vBot()
	# robot.connect()
	
	# for q in plan:
	# 	robot.move(q)
	# 	time.sleep(1)

	# robot.destroy()

else:
	print("No solution found")



