import numpy as np
import math
import time
import random

import RobotUtil as rt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Locobot:

	def __init__(self):
		# Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
		random.seed(8)

		self.Rdesc=[
			[0, 0, 0, 0, 0, 0.333],
			[-np.pi/2, 0, 0, 0, 0, 0],
			[np.pi/2, 0, 0, 0, -0.316, 0],
			[np.pi/2, 0, 0, 0.0825, 0, 0],
			[-np.pi/2, 0, 0, -0.0825, 0.384, 0],
			[np.pi/2, 0, 0, 0, 0, 0],
			[np.pi/2, 0, 0, 0.088, 0, 0],
			[0,0,0, 0,0,0.107]
			]
			
		#Define the axis of rotation for each joint
		self.axis=[
					[0, 0, 1],
					[0, 0, 1],
					[0, 0, 1],
					[0, 0, 1],
		   		[0, 0, 1],
			[0, 0, 1],
			[0, 0, 1],
			[0, 0, 1]
				]

		#Set base coordinate frame as identity - NOTE: don't change
		self.Tbase= [[1,0,0,0],
			[0,1,0,0],
			[0,0,1,0],
			[0,0,0,1]]
		
		#Initialize matrices - NOTE: don't change this part
		self.Tlink=[] #Transforms for each link (const)
		self.Tjoint=[] #Transforms for each joint (init eye)
		self.Tcurr=[] #Coordinate frame of current (init eye)
		for i in range(len(self.Rdesc)):
			self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3],self.Rdesc[i][3:6]))
			self.Tcurr.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			self.Tjoint.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])

		self.Tlinkzero=rt.rpyxyz2H(self.Rdesc[0][0:3],self.Rdesc[0][3:6])
		self.Tlink[0]=np.matmul(self.Tbase,self.Tlink[0])

		# initialize Jacobian matrix
		self.J=np.zeros((6,7))
		
		self.q=[0.,0.,0., 0.,0.,0., 0.,0.]
		self.ForwardKin([0.,0.,0., 0.,0.,0., 0.,0.])


		# NEW #############################
		# joint limits		
		

		self.qmin=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
		self.qmax=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]

		#Robot collision blocks descriptor (base frame, (rpy xyz), lwh 
		# NOTE: Cidx and Cdesc are just the robot's link BB's 
		#Robot collision blocks descriptor (base frame, (rpy xyz), lwh
		self.Cidx=[0,1,2,3,4,4,5,6, 7]
		self.Cdesc=[
		[0.,0.,0., 0.,0.,0.15-0.333],
		[0.,0.,0., 0.,0.,0.],
		[0.,0.,0., 0.,0.,0.175-0.316],
		[0.,0.,0., 0.,0.,0.],
		[0.,0.,0., 0.,0.,-0.25],
		[0.,0.,0., 0.,0.075,-0.09],
		[0.,0.,0., 0.,0.,0.],
		[0.,0.,0., 0.,0.,0.02],
		[0. , 0., -0.7853982, 0.0136,  -0.0092,  0.1457]
		]

		self.Cdim=[
		[0.15,0.15,0.3],
		[0.15,0.175,0.275],
		[0.125,0.125,0.275],
		[0.135,0.135,0.225],
		[0.125,0.125,0.25],
		[0.125,0.125,0.275],
		[0.125,0.125,0.125],
		[0.1,0.1,0.2],
		[0.08, 0.22, 0.17]
		]


		self.Tblock=[] #Transforms for each arm block
		self.Tcoll=[]  #Coordinate frame of current collision block

		self.Cpoints=[]
		self.Caxes=[]

		for i in range(len(self.Cdesc)):
			self.Tblock.append(rt.rpyxyz2H(self.Cdesc[i][0:3],self.Cdesc[i][3:6]))
			self.Tcoll.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			
			self.Cpoints.append(np.zeros((3,4)))
			self.Caxes.append(np.zeros((3,3)))


	def ForwardKin(self,ang):
		'''
		inputs: joint angles
		outputs: joint transforms for each joint, Jacobian matrix
		'''
		self.q[0:-1]=ang
		
		#Compute current joint and end effector coordinate frames (self.Tjoint). Remember that not all joints rotate about the z axis!
		for i in range(len(self.Rdesc)):
			if self.axis[i] == [0,0,1]:
				self.Tjoint[i]=[[math.cos(self.q[i]),-math.sin(self.q[i]),0,0],[math.sin(self.q[i]),math.cos(self.q[i]),0,0],[0,0,1,0],[0,0,0,1]]
			elif self.axis[i] == [-1,0,0]:
				self.Tjoint[i]=[[1,0,0,0],[0,math.cos(self.q[i]), math.sin(self.q[i]),0],[0,-math.sin(self.q[i]),math.cos(self.q[i]),0],[0,0,0,1]]
			elif self.axis[i] == [0,1,0]:
				self.Tjoint[i]=[[math.cos(self.q[i]),0,math.sin(self.q[i]),0],[0,1,0,0],[-math.sin(self.q[i]),0,math.cos(self.q[i]),0],[0,0,0,1]]
			else:
				raise ValueError('Axis rotation is not defined')

			if i is 0:
				self.Tcurr[i]=np.matmul(self.Tlink[i],self.Tjoint[i])
			else:
				self.Tcurr[i]=np.matmul(np.matmul(self.Tcurr[i-1],self.Tlink[i]),self.Tjoint[i])
		
		# Compute Jacobian matrix		
		for i in range(len(self.Tcurr)-1):
			p=self.Tcurr[-1][0:3,3]-self.Tcurr[i][0:3,3]
			# tells which axis the joint is rotating about
			ax_of_rotation = np.nonzero(self.axis[i])[0][0]	
			a=self.Tcurr[i][0:3,ax_of_rotation]*np.sign(self.axis[i][ax_of_rotation])
			self.J[0:3,i]=np.cross(a,p)
			self.J[3:7,i]=a

		return self.Tcurr, self.J




	def IterInvKin(self,ang,TGoal,x_eps=1e-3, r_eps=1e-3):
		'''
		inputs: starting joint angles (ang), target end effector pose (TGoal)

		outputs: computed joint angles to achieve desired end effector pose, 
		Error in your IK solution compared to the desired target
		'''	
		step_size_r = 0.5
		step_size_p = 0.02
		
		self.ForwardKin(ang)
		
		Err=[0.,0.,0.,0.,0.,0.] # error in position and orientation, initialized to 0
		for s in range(1000):
			#Compute rotation error
			rErrR=np.matmul(TGoal[0:3,0:3],np.transpose(self.Tcurr[-1][0:3,0:3]))
			rErrAxis,rErrAng=rt.R2axisang(rErrR)
			if rErrAng>step_size_r:
				rErrAng=step_size_r
			if rErrAng<-step_size_r:
				rErrAng=-step_size_r
			rErr= [rErrAxis[0]*rErrAng,rErrAxis[1]*rErrAng,rErrAxis[2]*rErrAng]
			Err[3:6]=rErr			

			#Compute position error
			xErr=TGoal[0:3,3]-self.Tcurr[-1][0:3,3]
			if np.linalg.norm(xErr)>step_size_p:
				xErr= xErr*step_size_p/np.linalg.norm(xErr)
			Err[0:3]=xErr
			
			# if norm of the error is below thresholds, then exit 
			if np.linalg.norm(Err[0:3]) <= x_eps and np.linalg.norm(Err[3:6]) <= r_eps:
				print("found IK solution with err:",np.linalg.norm(Err[0:3]),np.linalg.norm(Err[3:6]))
				break

			#Update joint angles
			C=np.diag([1e-6,1e-6,1e-6,1e-6,1e-6,1e-4])
			self.q[0:-1]=self.q[0:-1]+np.matmul(np.matmul(np.transpose(self.J[0:6,:]),np.linalg.inv(C+np.matmul(self.J[0:6,:],np.transpose(self.J[0:6,:])))),Err)
		

			#Recompute forward kinematics for new angles
			self.ForwardKin(self.q[0:-1])

		
		return self.q[0:-1], Err


	def PlotSkeleton(self,ang):
		#Compute forward kinematics for ang
		self.ForwardKin(ang)

		#Create figure
		fig =plt.figure()
		ax = fig.add_subplot(111,projection='3d')

		#Draw links along coordinate frames 
		for i in range(len(self.Tcurr)):
			ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
			if i is 0:
				ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='b')
			else:
				ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], [self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], [self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

		#Format axes and display
		ax.axis('equal')
		ax.set(xlim=(-1., 1.), ylim=(-1., 1.), zlim=(0,1.))
		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		plt.show()

	# NEW #############################
	def SampleRobotConfig(self):
		q=[]
		for i in range(7):
			q.append(self.qmin[i]+(self.qmax[i]-self.qmin[i])*random.random())
		return q


	def CompCollisionBlockPoints(self,ang):
		self.ForwardKin(ang)

		#TODO-Compute current collision boxes for arm 
		for i in range(len(self.Cdesc)):
			self.Tcoll[i] = np.matmul(self.Tcurr[self.Cidx[i]], self.Tblock[i])
			self.Cpoints[i],self.Caxes[i]=rt.BlockDesc2Points(self.Tcoll[i], self.Cdim[i])
		
	def DetectCollision(self, ang, pointsObs, axesObs):		
		self.CompCollisionBlockPoints(ang)

		for i in range(len(self.Cpoints)):
			for j in range(len(pointsObs)):
				if rt.CheckBoxBoxCollision(self.Cpoints[i],self.Caxes[i],pointsObs[j], axesObs[j]):
					return True
		return False


	def DetectCollisionEdge(self, ang1, ang2, pointsObs, axesObs):
		for s in np.linspace(0,1,5): #The number of steps along the edge may need to be adapted
			ang= [ang1[k]+s*(ang2[k]-ang1[k]) for k in range(len(ang1))] 	

			self.CompCollisionBlockPoints(ang)

			for i in range(len(self.Cpoints)):
				for j in range(len(pointsObs)):
					if rt.CheckBoxBoxCollision(self.Cpoints[i],self.Caxes[i],pointsObs[j], axesObs[j]):
						return True

		return False

	def PlotCollisionBlockPoints(self,ang,pointsObs=[]):
		#Compute collision block points
		self.CompCollisionBlockPoints(ang)
		#Create figure
		fig =plt.figure()
		ax = fig.add_subplot(111,projection='3d')


		#Draw links along coordinate frames 
		for i in range(len(self.Tcurr)):
			ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
			if i is 0:
				ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='k')
			else:
				ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], 
					[self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], 
					[self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

		for b in range(len(self.Cpoints)):
			for i in range(1,9): #TODO might have to change the 9 to 5?
				for j in range(i,9):
					ax.plot([self.Cpoints[b][i][0],  self.Cpoints[b][j][0]], 
						[self.Cpoints[b][i][1],  self.Cpoints[b][j][1]], 
						[self.Cpoints[b][i][2],  self.Cpoints[b][j][2]], c='b')
				
		for b in range(len(pointsObs)):
			for i in range(1,9):
				for j in range(i,9):
					ax.plot([pointsObs[b][i][0],  pointsObs[b][j][0]], 
						[pointsObs[b][i][1],  pointsObs[b][j][1]], 
						[pointsObs[b][i][2],  pointsObs[b][j][2]], c='r')


		#Format axes and display
		#ax.axis('equal')
		ax.set(xlim=(-0.6, .6), ylim=(-0.6, 0.6), zlim=(0,1.2))
		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		plt.show()
		return fig, ax


	


