import ThreeDimCurve

class Tube:

	def __init__(self):
		self.xFunc=lambda t: t
		self.yFunc=lambda t: 0
		self.zFunc=lambda t: 0
		self.lineCurve=ThreeDimCurve(xFunc,yFunc,zFunc)
		self.equation=