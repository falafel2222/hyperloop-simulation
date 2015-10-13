from sympy import *
import numpy as np

class ThreeDimCurve:
	def __init__(self,xFunc,yFunc,zFunc):
		self.xFunc=xFunc
		self.yFunc=yFunc
		self.zFunc=zFunc
		self.eqnPos=[xFunc,yFunc,zFunc]

	def evaluateVectorAtParam(self,param,vector,symbol=Symbol('t')):
		evaluated=map(lambda x: x.evalf(subs={symbol:param}),vector)
		return evaluated

	def getPositionAtParam(self,param):
		position=self.evaluateVectorAtParam(param,self.eqnPos)
		return position

	def getNormalAtParam(self,param):
		symbolicNormal=map(lambda x: x.diff(t),self.eqnPos)
		normalVector=self.evaluateVectorAtParam(param,symbolicNormal)
		return normalVector

	def getEqnOrthogonalPlaneAtParam(self,param):
		pointOnPlane=self.getPositionAtParam(param)
		normalToPlane=self.getNormalAtParam(param)
		x=Symbol('x')
		y=Symbol('y')
		zConstant=reduce(lambda x,y: x+y,[a*b for a,b in zip(pointOnPlane,normalToPlane)])
		zConstant=zConstant/normalToPlane[2]
		zPlane=(-1)*normalToPlane[0]*x+(-1)*normalToPlane[1]*y+zConstant
		return zPlane



t=Symbol('t')
x=t**2
y=t+2
z=1*t

c=ThreeDimCurve(x,y,z)

print c.getPositionAtParam(2)
print c.getNormalAtParam(2)
print c.getEqnOrthogonalPlaneAtParam(2)



#print y.evalf(subs={x:2})'''