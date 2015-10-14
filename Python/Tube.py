from numpy import *
from SubTube import *
from Pod import *

import matplotlib.pyplot as plt

class Tube:


	def __init__(self,listOfSubTubes,pod):
		'''The tube is composed of several subtubes, all with constant length and curvature,
		where curvature is defined as 1/r. At initialization, the total length of the overall
		tube is calculated from the total length of all constituent subtubes.'''
		#all subtubes which compose the overall tube
		self.constituents=listOfSubTubes
		#total length of the overall tube
		#self.length=reduce(lambda x,y:x.length+y.length,listOfSubTubes)
		self.length=0
		for subtube in self.constituents:
			self.length+=subtube.length
		self.pod=pod

	def radiusOfTube(self,theta):
		'''Defines the radius of the tube at any given theta, given in radians. 
		Values derived from tube radius of 35 in, with perpendicular height
		of bottom plane of 23 in.'''
		#reduce angles into the 0 to 2pi range.
		while theta>2*pi:
			theta-=2*pi
		while theta<0:
			theta+=2*pi

		#the alpha angle is the angle until the flat bottom of the tube starts
		alphaAngle=3*pi/2-arccos(23.0/35) # 3.85861463129
		#the beta angle is the angle until the flat bottom of the tube ends
		betaAngle=3*pi/2+arccos(23.0/35) # 5.56616332948
		if theta>alphaAngle and theta<betaAngle:
			#geometric determination of radius to the flat bottom
			gammaAngle=3*pi/2-theta
			radiusAtTheta=inToM(35.0)/cos(gammaAngle)
		else:
			#radius defined at 35 inches outside of the flat bottom
			radiusAtTheta=inToM(35.0)
		return radiusAtTheta

	def isPointOutsideTube(self,point):
		'''Given an (x,y,z) point, checks if the points are outside the tube
		at their determined theta.'''
		xValue,yValue,zValue=point
		#Converts to polar for tube radius checking
		radialDistance,theta=convertToPolar(yValue,zValue)
		radiusOfTubeAtTheta=self.radiusOfTube(theta)
		#A positive distance differential implies the point is outside the tube, and
		#has the collided.
		distanceDifferential=radialDistance-radiusOfTubeAtTheta
		return [distanceDifferential>=0,distanceDifferential]

	def checkPointsForCollisions(self,points):
		'''Checks all given points for collision with wall in previous
		update step.'''
		pointsAndCollisions=[(point,self.isPointOutsideTube(point)) \
		for point in points if self.isPointOutsideTube(point)[0]]
		return pointsAndCollisions

	def checkForCompletion(self,points):
		'''Checks if any points have completely traversed the loop'''
		completions=[point for point in points if point[0]>=self.length]
		return completions!=[]

	def findPointSubtube(self,point):
		lengths=[x.length for x in self.constituents]
		tempXVal=point[0]
		for i in range(len(lengths)):
			if tempXVal>=lengths[i]:
				tempXVal-=lengths[i]
			else:
				return self.constituents[i]
		return self.constituents[-1]

	def updateStep(self):
		'''Updates the tube model, checking the tube has been completed,
		if any collisions have occured, and providing points with the 
		centripetal acceleration due to them based upon their subtube.'''
		points,velTangent=self.pod.getData()

		collisions=self.checkPointsForCollisions(points)

		pointsAndSubTubes=[(point,self.findPointSubtube(point)) for point in points]
		accelerations=[(point,subtube.centripetalAccelInSubTube(velTangent)) for \
		(point,subtube) in pointsAndSubTubes]
		accel=reduce(lambda x, y: x + y, accelerations)
		self.pod.update(accel)
		return collisions,points



	def beginUpdates(self):
		'''Begins the update steps'''
		points,velTangent=self.pod.getData()
		collisions=self.checkPointsForCollisions(points)
		i=0

		xPosList = []
		xVelList = []
		xAccelList = []
		timeList = []

		while collisions==[] and not self.checkForCompletion(points):
	
			if i % 1000 == 0:
				print i*TIMESTEP, points[0][0]

			# if points[0][0] > 500:
			# 	self.pod.startBraking()

			timeList.append(i*TIMESTEP)
			
			xPosList.append(self.pod.position[0])
			xVelList.append(self.pod.velocity[0])	
			xAccelList.append(self.pod.accel[0])

			collisions,points=self.updateStep()
			i+=1

		figure = plt.figure()
		figure.suptitle("Graph for drag coefficient = " + str(DRAG_COEFFICIENT))

		plt.subplot(3,1,1)
		plt.plot(timeList, xPosList)
		plt.ylim([0,self.length])
		plt.ylabel('Position of Pod(m)')
		plt.xlabel('Time (s)')
		

		plt.subplot(3,1,2)
		plt.plot(timeList, xVelList)
		plt.ylabel('Velocity of Pod(m)')
		plt.xlabel('Time (s)')
		
		plt.subplot(3,1,3)
		plt.plot(timeList, xAccelList)
		plt.ylabel('Acceleration of Pod(m)')
		plt.xlabel('Time (s)')

		plt.show()

		if collisions!=[]:
			print "uh oh"
			print collisions
		if self.checkForCompletion(points):
			print points
			print "Hooray"

def convertToPolar(yValue,zValue):
	if yValue==0:
		if zValue>0:
			theta=pi/2
		if zValue<=0:
			theta=-1*pi/2
	else:
		theta=arctan(zValue*1.0/yValue)
	radialDistance=sqrt(yValue**2+zValue**2)
	return [radialDistance,theta]

def inToM(inches):
	return inches/39.370

def main():
	sub1=SubTube(1600,0,False)
	subs=[sub1]
	pod=Pod()
	tube=Tube(subs,pod)
	tube.beginUpdates()

if __name__ == '__main__':
	main()