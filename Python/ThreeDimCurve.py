class ThreeDimCurve:
	def __init__(self,xFunc,yFunc,zFunc):
		self.xFunc=xFunc
		self.yFunc=yFunc
		self.zFunc=zFunc

	def getPosition(self,param):
		position=[self.xFunc(param),self.yFunc(param),self.zFunc(param)]
		return position

class Ellipse(ThreeDimCurve):
	def __init__(self,yFunc,zFunc):
		super(Ellipse,)
