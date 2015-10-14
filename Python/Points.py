#deprecated

'''class Point:
	def __init__(self,xyzValues):
		self.xVal,self.yVal,self.zVal=xyzValues
		self.subTube=None

	def findSubTube(self,listOfSubTubes):
		lengths=[x.length for x in listOfSubTubes]
		tempXVal=self.xVal
		for i in range(len(lengths)):
			if tempXVal>=lengths[i]:
				tempXVal-=lengths[i]
			else:
				return i
		return False'''