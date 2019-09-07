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

	def d2B(self,x):
		hx = self.h(x)
		if hx == 0:
			hx = 1e-3
		return -1.0/(hx**3)*(self.d2h(x) -2*np.matmul(self.dh(x),self.dh(x).T))


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

	def d2h(self,x):
		return np.array([[0],[0],[0],[0]])

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
		return np.stack((np.array([0]),np.array([0]),2*z[2,:],2*z[3,:]))*z[4,:]*sgn

	def d2h(self,z):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0
		return np.block([[np.zeros((2,2)),np.zeros((2,2))],[np.zeros((2,2)),2*sgn*z[4,:]*np.ones((2,2))]])


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
		z1 = z[0,:]
		z2 = z[1,:]
		z3 = z[2,:]
		z4 = z[3,:]
		z5 = z[4,:]
		gamma_p = self.gamma_p
		x_pos = self.x
		y_pos = self.y
		return sgn * np.stack((
			(z3*(d**2) - x_pos**2*z3 - z1**2*z3 - gamma_p*x_pos*(d**2) + gamma_p*z1*(d**2) - x_pos*y_pos*z4 + 2*x_pos*z1*z3 + x_pos*z2*z4 + y_pos*z1*z4 - z1*z2*z4)/(d**3),
			(z4*(d**2) - y_pos**2*z4 - z2**2*z4 - gamma_p*y_pos*(d**2) + gamma_p*z2*(d**2) - x_pos*y_pos*z3 + x_pos*z2*z3 + y_pos*z1*z3 + 2*y_pos*z2*z4 - z1*z2*z3)/(d**3),
			-(x_pos - z1)/d,
			-(y_pos - z2)/d))

	def d2h(self,z):
		sgn = 1.0
		if self.radius < 0.0:
			sgn = -1.0

		d = np.sqrt((z[0,:] - self.x)**2 + (z[1,:] - self.y)**2) + 1.0e-6
		z1 = z[0,:]
		z2 = z[1,:]
		z3 = z[2,:]
		z4 = z[3,:]
		z5 = z[4,:]
		gamma_p = self.gamma_p
		x_pos = self.x
		y_pos = self.y
		return sgn * np.block([
			[ ((y_pos - z2)*(gamma_p*x_pos**2*y_pos - gamma_p*x_pos**2*z2 - 2*z4*x_pos**2 - 2*gamma_p*x_pos*y_pos*z1 + 3*z3*x_pos*y_pos + 2*gamma_p*x_pos*z1*z2 + 4*z4*x_pos*z1 - 3*z3*x_pos*z2 + gamma_p*y_pos**3 - 3*gamma_p*y_pos**2*z2 + z4*y_pos**2 + gamma_p*y_pos*z1**2 - 3*z3*y_pos*z1 + 3*gamma_p*y_pos*z2**2 - 2*z4*y_pos*z2 - gamma_p*z1**2*z2 - 2*z4*z1**2 + 3*z3*z1*z2 - gamma_p*z2**3 + z4*z2**2))/(d**2)**(5/2),(z4*(x_pos - z1))/(d**2)**(3/2) + (z3*(y_pos - z2))/(d**2)**(3/2) - (gamma_p*(2*x_pos - 2*z1)*(2*y_pos - 2*z2))/(4*(d**2)**(3/2)) - (3*z3*(2*x_pos - 2*z1)*(2*y_pos - 2*z2)*(x_pos - z1))/(4*(d**2)**(5/2)) - (3*z4*(2*x_pos - 2*z1)*(2*y_pos - 2*z2)*(y_pos - z2))/(4*(d**2)**(5/2)),(y_pos - z2)**2/(d**2)**(3/2), -((2*x_pos - 2*z1)*(y_pos - z2))/(2*(d**2)**(3/2))],
			[(z4*(x_pos - z1))/(d**2)**(3/2) + (z3*(y_pos - z2))/(d**2)**(3/2) - (gamma_p*(2*x_pos - 2*z1)*(2*y_pos - 2*z2))/(4*(d**2)**(3/2)) - (3*z3*(2*x_pos - 2*z1)*(2*y_pos - 2*z2)*(x_pos - z1))/(4*(d**2)**(5/2)) - (3*z4*(2*x_pos - 2*z1)*(2*y_pos - 2*z2)*(y_pos - z2))/(4*(d**2)**(5/2)), ((x_pos - z1)*(gamma_p*x_pos**3 - 3*gamma_p*x_pos**2*z1 + z3*x_pos**2 + gamma_p*x_pos*y_pos**2 - 2*gamma_p*x_pos*y_pos*z2 + 3*z4*x_pos*y_pos + 3*gamma_p*x_pos*z1**2 - 2*z3*x_pos*z1 + gamma_p*x_pos*z2**2 - 3*z4*x_pos*z2 - gamma_p*y_pos**2*z1 - 2*z3*y_pos**2 + 2*gamma_p*y_pos*z1*z2 - 3*z4*y_pos*z1 + 4*z3*y_pos*z2 - gamma_p*z1**3 + z3*z1**2 - gamma_p*z1*z2**2 + 3*z4*z1*z2 - 2*z3*z2**2))/(d**2)**(5/2), -((2*y_pos - 2*z2)*(x_pos - z1))/(2*(d**2)**(3/2)),(x_pos - z1)**2/(d**2)**(3/2)],
			[(y_pos - z2)**2/(d**2)**(3/2),-((2*y_pos - 2*z2)*(x_pos - z1))/(2*(d**2)**(3/2)),0,0],
			[-((2*x_pos - 2*z1)*(y_pos - z2))/(2*(d**2)**(3/2)),(x_pos - z1)**2/(d**2)**(3/2),0,0]])