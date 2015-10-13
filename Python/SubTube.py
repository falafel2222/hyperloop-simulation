class SubTube:

	def __init__(self,length,curvature,righthanded):
		self.length=length
		self.curvature=curvature
		self.righthanded=righthanded
		self.radius=1.0/curvature if curvature>0 else None

	def centripetalAccelInSubTube(self,velocity):
		if self.radius==None:
			return 0
		else:
			accelMag=(velocity**2)/self.radius
			accel=accelMag if self.righthanded else -1*accelMag
