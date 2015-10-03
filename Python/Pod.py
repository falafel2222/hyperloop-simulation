# pod class for hyperloop simulation

# units are SI (meters, seconds, kilograms)

TIMESTEP = .001

from Force import Force;

class Pod:
	def __init__(self):

		# rough numbers on pod dimensions due to spaceX rules
		self.baseWidth = 1.1
		self.midwidth = 1.3
		self.height = 1.2
		self.length = 4.0

		# all of these vectors are of the form [x,y,z]
		self. accel = [0.0,0.0,0.0]
		self.velocity = [0.0,0.0,0.0]
		self.position = [self.length,0.0,0.0]

		# rotation is a vector of roll, pitch, yaw (about x, y and z)
		self.rotation = [0.0,0.0,0.0]

		# mass and center of mass are needed to caclulate effects of forces
		self.centerOfMass = [-self.length,0.0,-.2]
		self.mass = 3100.0;

		self.forces = []


	"""
		@param vector:  vector of x,y,z force
		@param location:  vector of x,y,z where the force is applied
		@param duration: how long the force should last
	"""
	def applyForce(self, vector, location, duration):
		# add the force to the list

		f = Force(vector, location, duration)
		self.forces.append(f);


	"""
		update function, this updates the pod's states over one timestep
	"""
	def update(self):
		for i in range(len(self.forces)):

			# if the force is done, remove it
			if self.forces[i].timeOver():
				self.forces.remove(i)
			else:
				vector, location = self.forces[i].useForce(TIMESTEP)



def main():

	p = Pod()

	p.applyForce([2,0,0],[-4,0,-.2],1)

	p.update()


if __name__ == "__main__":
	main()