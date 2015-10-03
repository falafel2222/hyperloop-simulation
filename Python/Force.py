class Force:
	def __init__(self, vector, location, time):

		self.vector = vector
		self.location = location
		self.timeLeft = time;


	"""
		if the force is done being applied, return false
	"""
	def timeOver(self):
		if self.timeLeft < 0:
			return False;
	
	"""
		decrements the time remaining by timeStep
		and returns a tuple of form (vector, location)
	"""
	def useForce(self, timeStep):

		self.timeLeft -= timeStep
		return self.vector, self.location