# pod class for hyperloop simulation

# units are SI (meters, seconds, kilograms)

import numpy
import math
import random

# Relevant Constants

TIMESTEP = .001

AIR_PRESSURE = .1 # psi

DRAG_COEFFICIENT = 0.2
AIR_DENSITY = 6900*AIR_PRESSURE/(287*298)

PUSHER_FORCE = 23520 # Newtons
PUSHER_DISTANCE = 243 #meters

COM_Z_DROP = -.2

class Pod:
	def __init__(self):

		self.time = 0

		self.braking = False


		# rough numbers on pod dimensions due to spaceX rules
		self.baseWidth = 1.1
		self.midwidth = 1.3
		self.height = 1.2
		self.length = 4.0

		# all of these vectors are of the form [x,y,z]
		self. accel = [0.0,0.0,0.0]
		self.velocity = [0.0,0.0,0.0]
		self.position = [self.length/2, 0.0, COM_Z_DROP]

		# rotation is a vector of roll, pitch, yaw (about x, y and z)
		self.rotPos = [1,1,1]
		self.rotVel = [0.0,0.0,0.0]
		self.rotAccel = [0.0,0.0,0.0]


		# mass and center of mass are needed to caclulate effects of forces
		self.centerOfMass = [0,0,0]
		self.mass = 3000.0 # Kilograms

		# moment of inertia of the pod (about x, y, and z)
		self.momentOfInertia = [0,0,0]

		self.momentOfInertia[0] = 1/8.0*self.mass*self.midwidth * self.height
		self.momentOfInertia[1] = 1/12.0*self.mass*(self.length**2)
		self.momentOfInertia[2] = 1/12.0*self.mass*(self.length**2)


		# upwards air thruster positions.

		self.upwardsThrusters = []
		self.upwardsThrusters.append((0,self.baseWidth))

		self.collisionPoints = []

		# front 5 points (3 for now)
		self.collisionPoints.append([self.length/2, self.baseWidth/2, -self.height/2 - COM_Z_DROP])
		self.collisionPoints.append([self.length/2, -self.baseWidth/2, -self.height/2 - COM_Z_DROP])
		self.collisionPoints.append([self.length/2, 0, self.height/2 - COM_Z_DROP])

		# back 5 points (3 for now)
		self.collisionPoints.append([-self.length/2, self.baseWidth/2, -self.height/2 - COM_Z_DROP])
		self.collisionPoints.append([-self.length/2, -self.baseWidth/2, -self.height/2 - COM_Z_DROP])
		self.collisionPoints.append([-self.length/2, 0, self.height/2 - COM_Z_DROP])


	"""
		update function, this updates the pod's states over one timestep
	"""
	def update(self, aCentrip):

		self.accel = [0.0,0.00,0.0]

		# Update the forces out of our control

		# pusher force
		if self.position[0] < PUSHER_DISTANCE:
			self.accel[0] += PUSHER_FORCE / self.mass

		# drag force
		self.accel[0] -= DRAG_COEFFICIENT * AIR_DENSITY * self.baseWidth * self.height * self.velocity[0]**2 / (2.0*self.mass)

		# gravity 
		self.accel[2] -= 9.8


		if self.braking:
			self.accel[0] -= 5 * AIR_DENSITY * self.baseWidth * self.height * self.velocity[0]**2 / (2.0*self.mass)

		# apply upwards air thrusters with randomness and simple PID
		self.accel[2] += 9.8 + .0005*(random.random()-.5)

		# self.accel[2] += (- .02 * (self.position[2]-COM_Z_DROP) + .0005*self.velocity[2])


		# update velocity based on acceleration
		self.velocity = [self.velocity[i] + TIMESTEP*self.accel[i] for i in range(3)]

		# update position based on velocity
		self.position = [self.position[i] + TIMESTEP*self.velocity[i] for i in range(3)]

		# same with rotational velocity and position
		self.rotVel = [self.rotVel[i] + TIMESTEP * self.rotAccel[i] for i in range(3)]
		self.rotPos = [self.rotPos[i] + TIMESTEP * self.rotVel[i] for i in range(3)]

		self.time += TIMESTEP



	"""
		method to return data for use in the tube class
		returns a tuple of (transformed collision points, x velocity)
	"""
	def getData(self):
		transformedPoints = []

		# rotation matrices about x, y and z

		rotMatrix1 = [	[1,0,0],
						[0,math.cos(self.rotPos[0]), -math.sin(self.rotPos[0])],
						[0,math.sin(self.rotPos[0]), math.cos(self.rotPos[0])]]
		
		rotMatrix2 = [	[math.cos(self.rotPos[1]), 0, math.sin(self.rotPos[1])],
						[0,1,0],
						[-math.sin(self.rotPos[1]),0, math.cos(self.rotPos[1])]]
		
		rotMatrix3 = [	[math.cos(self.rotPos[2]), -math.sin(self.rotPos[2]),0],
						[math.sin(self.rotPos[2]), math.cos(self.rotPos[2]),0],
						[0,0,1]]


		# multiply the three rotation matrices together to get a total rotation matrix
		totRotMatrix = numpy.inner(rotMatrix1, rotMatrix2)
		totRotMatrix = numpy.inner(rotMatrix3, totRotMatrix)

		# print totRotMatrix


		for point in self.collisionPoints:

			# point = numpy.inner(totRotMatrix, point)

			point = [point[i] + self.position[i] for i in range(3)]

			transformedPoints.append(point)

		return transformedPoints, self.velocity[0]


	def startBraking(self):
		self.braking = True 

	def stopBraking(self):
		self.braking = False



def main():

	p = Pod()

	print p.getData()
	
	for i in range(60*1000):
		p.update(0)
		if i%1000 == 0:
			print "Position", 
			print p.position[2],
			print ", Velocity",
			print p.velocity[2],			
			print ", Acceleration",
			print  p.accel[2]


if __name__ == "__main__":
	main()