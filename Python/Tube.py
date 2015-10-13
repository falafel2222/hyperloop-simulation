from numpy import *
from SubTube import *
from Points import Point

class Tube:

	def __init__(self,listOfSubTubes):
		self.constituents=listOfSubTubes
		self.length=reduce(lambda x,y:x.length+y.length,listOfSubTubes)
		self.totalDistanceToEndOfCurrent=self.currentTube.length
		self.time=0.0
		self.timeInterval=1
		self.velocity=20

	def radiusOfTube(self,theta):
		'''Defines the radius of the tube at any given theta, given in radians'''
		#Values derived from tube radius of 35 in, with perpendicular height
		#of bottom plane of 23 in.
		while theta>2*pi:
			theta-=2*pi
		while theta<0:
			theta+=2*pi
		alphaAngle=3*pi/2-arccos(23.0/35) # 3.85861463129
		betaAngle=3*pi/2+arccos(23.0/35) # 5.56616332948
		if theta>alphaAngle and theta<betaAngle:
			gammaAngle=3*pi/2-theta
			radiusAtTheta=23.0/cos(gammaAngle)
		else:
			radiusAtTheta=35.0
		return radiusAtTheta

	def isPointOutsideTube(self,point):
		xValue,yValue,zValue=point
		radialDistance,theta=convertToPolar(yValue,zValue)
		radiusOfTubeAtTheta=self.radiusOfTube(theta)
		distanceDifferential=radialDistance-radiusOfTubeAtTheta
		return [distanceDifferential>=0,distanceDifferential]

	def checkPointsForCollisions(self,points):
		pointsAndCollisions=[(point,self.isPointOutsideTube(point)) for point in points if self.isPointOutsideTube(point)[0]]
		return pointsAndCollisions

	def checkForCompletion(self,points):
		return any(map(lambda x: x[0], points)>=self.length)

	def updateStep(self,points):
		pointsToCheck=[]
		for pointCoordinates in points:
			newPoint=Point(pointCoordinates)
			newPoint.findSubTube(self.constituents)
			pointsToCheck.append(newPoint)

		if self.checkForCompletion(points):
			print "Hooray"
			return 0
		elif self.checkPointsForCollisions(points)!=[]:
			print "uh oh!"
			return self.checkPointsForCollisions(points)
		time+=timeInterval

		#Pod is updated

def convertToPolar(yValue,zValue):
	theta=arctan(zValue*1.0/yValue)
	radialDistance=sqrt(yValue**2+zValue**2)
	return [radialDistance,theta]

def main():
	sub1=SubTube(100,0,False)
	sub2=SubTube(100,0.0001,False)
	subs=[sub1,sub2]
	tube=Tube(subs)
	points=[(200,10,10),(15,-10,-10),(17,3,3)]
	tube.updateStep(points)

if __name__ == '__main__':
	main()