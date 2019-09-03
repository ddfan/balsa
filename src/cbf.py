import numpy as np

class Barrier():
	def __init__(self,dim,gamma):
		self.dim=dim
		self.gamma = gamma

	def h(self,x):
		# overload
		return None

	def dh(self,x):
		# overload
		return None

	def B(self,x):
		# TODO:  parameterize this, and craft something better.
		hx = self.h(x)
		# return np.exp(-hx+1)
		if hx == 0:
			hx = 1e-3
		return 1.0/hx

	def dB(self,x):
		hx = self.h(x)
		if hx == 0:
			hx = 1e-3
		return -1.0/(hx**2)*self.dh(x)
		# return -np.exp(-hx+1)*self.dh(x)

class BarrierAckermannVelocity(Barrier):
	def __init__(self,dim=4,gamma=1.0,bound_from_above=True, v_lim = 1.0):
		self.bound_from_above = bound_from_above
		self.v_lim = v_lim

		Barrier.__init__(self,dim,gamma)

	def h(self,x):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0
		return sgn  * (x[3,:] - self.v_lim)

	def dh(self,x):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0
		return np.array([[0],[0],[0],[sgn]])


class BarrierAckermannPosition(Barrier):
	def __init__(self,dim=4,gamma=1.0, bound_from_above = True, bound_x = True, pos_lim = 1.0, gamma_p = 1.0):
		self.bound_from_above = bound_from_above
		self.bound_x = bound_x
		self.pos_lim = pos_lim
		self.gamma_p = gamma_p

		Barrier.__init__(self,dim,gamma)

	def h(self,x):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0

		if self.bound_x:
			return sgn * (self.gamma_p * (x[0,:] - self.pos_lim) + x[3,:] * np.cos(x[2,:]))
		else:
			return sgn * (self.gamma_p * (x[1,:] - self.pos_lim) + x[3,:] * np.sin(x[2,:]))
		 
	def dh(self,x):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0

		if self.bound_x:
			return sgn * np.stack((np.array([self.gamma_p]),np.array([0]),-x[3,:] * np.sin(x[2,:]),np.cos(x[2,:])),axis=0)
		else:
			return sgn * np.stack((np.array([0]),np.array([self.gamma_p]),x[3,:] * np.cos(x[2,:]),np.sin(x[2,:])),axis=0)

class BarrierAckermannVelocityZ(Barrier):
	def __init__(self,dim=4,gamma=1.0,bound_from_above=True, v_lim = 1.0):
		self.bound_from_above = bound_from_above
		self.v_lim = v_lim

		Barrier.__init__(self,dim,gamma)

	def h(self,z):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0
		return sgn  * (np.sqrt(z[2]**2 + z[3]**2) * z[4] - self.v_lim)

	def dh(self,z):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0
		v = (np.sqrt(z[2,:]**2 + z[3,:]**2) + 1.0e-6) * z[4,:]
		return np.stack((np.array([0]),np.array([0]),z[2,:]/v,z[3,:]/v))*sgn


class BarrierAckermannPositionZ(Barrier):
	def __init__(self,dim=4,gamma=1.0, bound_from_above = True, bound_x = True, pos_lim = 1.0, gamma_p = 1.0):
		self.bound_from_above = bound_from_above
		self.bound_x = bound_x
		self.pos_lim = pos_lim
		self.gamma_p = gamma_p

		Barrier.__init__(self,dim,gamma)

	def h(self,z):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0

		if self.bound_x:
			return sgn * (self.gamma_p * (z[0,:] - self.pos_lim) + z[2,:])
		else:
			return sgn * (self.gamma_p * (z[1,:] - self.pos_lim) + z[3,:])
		 
	def dh(self,z):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0

		if self.bound_x:
			return np.array([[sgn*self.gamma_p],[0],[sgn],[0]])
		else:
			return np.array([[0],[sgn*self.gamma_p],[0],[sgn]])


class BarrierAckermannPointZ(Barrier):
	def __init__(self,dim=4,gamma=1.0, x=0.0, y=0.0, radius = 1.0, gamma_p = 1.0):
		self.x = x
		self.y = y
		self.radius = radius
		self.gamma_p = gamma_p

		Barrier.__init__(self,dim,gamma)

	def h(self,z):
		sgn = 1.0
		if self.radius < 0.0:
			sgn = -1.0

		d = np.sqrt((z[0,:] - self.x)**2 + (z[1,:] - self.y)**2) + 1.0e-6
		return sgn * (self.gamma_p * (d - self.radius) + (z[0,:] - self.x) / d * z[2,:] + (z[1,:] - self.y) / d * z[3,:])
		 
	def dh(self,z):
		sgn = 1.0
		if self.radius < 0.0:
			sgn = -1.0

		d = np.sqrt((z[0,:] - self.x)**2 + (z[1,:] - self.y)**2) + 1.0e-6
		return sgn * np.stack(( self.gamma_p * (z[0,:] - self.x) / d - 1.0 / (d ** 3)*((z[0,:] - self.x)*z[2,:] + (z[1,:] - self.y)*z[3,:]) * (z[0,:] - self.x) + z[2,:] / d,
								self.gamma_p * (z[1,:] - self.y) / d - 1.0 / (d ** 3)*((z[0,:] - self.x)*z[2,:] + (z[1,:] - self.y)*z[3,:]) * (z[1,:] - self.y) + z[3,:] / d,
								(z[0,:] - self.x) / d,
								(z[1,:] - self.y) / d))