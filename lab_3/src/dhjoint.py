#!/usr/bin/env python

class djoint:
	def __init__(self,a,d,alfa,theta,rot):
		self.a=a
		self.d=d
		self.alfa=alfa
		self.theta=theta
		self.rot=rot
		if rot:		
			self.prev=theta
		else:
			self.prev=d
