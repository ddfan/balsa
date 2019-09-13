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
		# return -1.0/(hx**3)*(self.d2h(x) -2*np.matmul(self.dh(x),self.dh(x).T))
		# can ignore d2h because taking trace(G*sig*sig^T*G^T d2B).
		dh = self.dh(x)[2:].T
		return 1.0/(hx**3)*(2*np.outer(dh,dh))

	def get_B_derivatives(self,x):
		hx = self.h(x)
		if hx == 0:
			hx = 1e-3

		dh = self.dh(x)
		return hx, -1.0/(hx*hx)*dh.T, 2.0/(hx*hx*hx)*(np.outer(dh[2:],dh[2:]))


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
		z1 = z[0,:]
		z2 = z[1,:]
		z3 = z[2,:]
		z4 = z[3,:]
		z5 = z[4,:]
		v = (np.sqrt(z3**2 + z4**2) + 1.0e-6)
		return np.stack((np.array([0]), np.array([0]), (z3*z5)/v, (z4*z5)/v)) * sgn

	def d2h(self,z):
		sgn = 1.0
		if self.bound_from_above:
			sgn = -1.0
		z1 = z[0,:]
		z2 = z[1,:]
		z3 = z[2,:]
		z4 = z[3,:]
		z5 = z[4,:]
		v = (np.sqrt(z3**2 + z4**2) + 1.0e-6)
		H = np.block([[(z4**2*z5)/(v**3), -(z3*z4*z5)/(v**3)],
			[-(z3*z4*z5)/(v**3),   (z3**2*z5)/(v**3)]])
		return np.block([[np.zeros((2,2)),np.zeros((2,2))],[np.zeros((2,2)),H]], dtype=np.float32) * sgn

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

		d = np.sqrt((z[0] - self.x)*(z[0] - self.x) + (z[1] - self.y)*(z[1] - self.y)) + 1.0e-6
		return sgn * (self.gamma_p * (d - self.radius) + (z[0] - self.x) / d * z[2] + (z[1] - self.y) / d * z[3])
		 
	def dh(self,z):
		sgn = 1.0
		if self.radius < 0.0:
			sgn = -1.0

		d = np.sqrt((z[0] - self.x)*(z[0] - self.x) + (z[1] - self.y)*(z[1] - self.y)) + 1.0e-6
		d_2 = d*d
		d_3 = d*d*d
		y_pos_m_z2 = (self.y - z[1])
		x_pos_m_z1 = (self.x - z[0])
		return sgn * np.array((
			(z[2]*(d_2 - x_pos_m_z1*x_pos_m_z1) - self.gamma_p * d_2 *x_pos_m_z1 - z[3]*x_pos_m_z1*y_pos_m_z2)/d_3,
			(z[3]*(d_2 - y_pos_m_z2*y_pos_m_z2) - self.gamma_p * d_2 *y_pos_m_z2 - z[2]*x_pos_m_z1*y_pos_m_z2)/d_3,
			-x_pos_m_z1/d,
			-y_pos_m_z2/d), dtype=np.float32)

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
		y_pos_m_z2 = (y_pos - z2)
		x_pos_m_z1 = (x_pos - z1)
		d2 = d**2
		d3 = d**3
		d5 = d**5
		z1_2 = z1**2
		z2_2 = z2**2
		x_pos_2 = x_pos**2
		y_pos_2 = y_pos**2
		a11 = (y_pos_m_z2*(gamma_p *(x_pos_2*y_pos - x_pos_2*z2 + 3*y_pos*z2_2 - 2*x_pos*y_pos*z1 + 2*x_pos*z1*z2 + y_pos**3 - 3*y_pos_2*z2 + y_pos*z1_2 - z1_2*z2 - z2**3) - 2*z4*x_pos_2 + 3*z3*x_pos*y_pos + 4*z4*x_pos*z1 - 3*z3*x_pos*z2 + z4*y_pos_2 - 3*z3*y_pos*z1 - 2*z4*y_pos*z2 - 2*z4*z1_2 + 3*z3*z1*z2 + z4*z2_2))/(d5)
		a12 = x_pos_m_z1 * y_pos_m_z2 * (z4 + z3 - gamma_p - (3*z3*x_pos_m_z1)/(d2) - (3*z4*y_pos_m_z2)/(d2))/(d3)
		a13 = y_pos_m_z2**2/(d3)
		a14 = -(x_pos_m_z1*y_pos_m_z2)/(d3)
		a22 = (x_pos_m_z1*(gamma_p*(x_pos**3 - 3*x_pos_2*z1 + x_pos*y_pos_2 - 2*x_pos*y_pos*z2 + 3*x_pos*z1_2 + x_pos*z2_2 - y_pos_2*z1  + 2*y_pos*z1*z2 - z1**3 - z1*z2_2)+ z3*x_pos_2 + 3*z4*x_pos*y_pos - 2*z3*x_pos*z1 - 3*z4*x_pos*z2 - 2*z3*y_pos_2 - 3*z4*y_pos*z1 + 4*z3*y_pos*z2 + z3*z1_2 + 3*z4*z1*z2 - 2*z3*z2_2))/(d5)
		a23 = -(y_pos_m_z2*x_pos_m_z1)/(d3)
		a24 = x_pos_m_z1**2/(d3)

		return sgn * np.block([
			[ a11, a12, a13, a14],
			[ a12, a22, a23, a24],
			[ a13, a23, 0, 0],
			[ a14, a24, 0, 0]], dtype=np.float32)