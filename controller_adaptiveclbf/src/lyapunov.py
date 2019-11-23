import numpy as np

class Lyapunov():
	def __init__(self,dim,epsilon=1.0):
		self.dim=dim
		self.epsilon = epsilon

	def V(self,x):
		# overload
		return None

	def dV(self,x):
		# overload
		return None

	def signed_angle_dist(self,a,b):
		d = a - b
		if d > np.pi:
			d = d - np.pi*2
		elif d < -np.pi:
			d = d + np.pi*2

		return d

class LyapunovAckermann(Lyapunov):
	def __init__(self,dim=4,w1=1.0,w2=1.0,w3=1.0,epsilon=1e-6):
		self.w1=w1
		self.w2=w2
		self.w3=w3
		self.epsilon=epsilon
		Lyapunov.__init__(self,dim)

	def convert_to_polar(self,x,x_d):
		if x[3,:] < 0:
			e = -np.sqrt((x_d[0,:]-x[0,:])**2 + (x_d[1,:]-x[1,:])**2)
			phi_t = np.arctan2(x[1,:]-x_d[1,:],x[0,:]-x_d[0,:])
		else:
			e = np.sqrt((x_d[0,:]-x[0,:])**2 + (x_d[1,:]-x[1,:])**2)
			phi_t = np.arctan2(x_d[1,:]-x[1,:],x_d[0,:]-x[0,:])
		alpha = self.signed_angle_dist(phi_t,x[2,:])
		theta = self.signed_angle_dist(phi_t,x_d[2,:])

		# if alpha and theta are opposite signs, we have two confliciting headings.
		# Flip the sign of one to turn in one direction.
		# print('before:',alpha,theta)
		if np.sign(alpha) != np.sign(theta):
			if np.abs(alpha) >= np.abs(theta):
				alpha = alpha - np.sign(alpha) * 2 * np.pi
			else:
				theta = theta - np.sign(theta) * 2 * np.pi
		# print('after:',alpha,theta)
		# print(e,x[2,:]/(np.pi)*180,phi_t/(np.pi)*180,x_d[2,:]/(np.pi)*180,alpha/(np.pi)*180,theta/(np.pi)*180)

		return e,phi_t,alpha,theta

	def V(self,x,x_d):
		e,phi_t,alpha,theta = self.convert_to_polar(x,x_d)
		return 0.5 * (self.w1*alpha**2 + self.w2*theta**2 + self.w3*(x[3,:]-x_d[3,:])**2)

	def dV(self,x,x_d):
		e,phi_t,alpha,theta = self.convert_to_polar(x,x_d)
		c_tmp = (self.w1*alpha + self.w2*theta) / (e + self.epsilon)
		return np.stack((c_tmp*np.sin(phi_t),-c_tmp*np.cos(phi_t),-self.w1*alpha,self.w3*(x[3,:]-x_d[3,:])),axis=0)

class LyapunovAckermannZ(Lyapunov):
	def __init__(self,dim=4,w1=1.0, w2=1.0, w3=1.0, epsilon = 1e-6):
		self.w1=w1
		self.w2=w2
		self.w3=w3
		self.epsilon = epsilon
		Lyapunov.__init__(self,dim)


	def convert_to_polar(self,z,z_d):
		# if z[3,:] < 0:
		# 	e = -np.sqrt((z_d[0,:]-z[0,:])**2 + (z_d[1,:]-z[1,:])**2)
		# 	phi_t = np.arctan2(z[1,:]-z_d[1,:],z[0,:]-z_d[0,:])
		# else:
		e = np.sqrt((z_d[0,:]-z[0,:])**2 + (z_d[1,:]-z[1,:])**2)
		phi_t = np.arctan2(z_d[1,:]-z[1,:],z_d[0,:]-z[0,:])

		phi = np.arctan2(z[3,:],z[2,:])
		phi_d = np.arctan2(z_d[3,:],z_d[2,:])

		alpha = self.signed_angle_dist(phi_t,phi)
		theta = self.signed_angle_dist(phi_t,phi_d)
		# print('angles: ',phi,phi_t,phi_d,alpha,theta)
		# if alpha and theta are opposite signs, we have two confliciting headings.
		# Flip the sign of one to turn in one direction.
		# print('before:',alpha,theta)
		# if np.sign(alpha) != np.sign(theta):
		# 	if np.abs(alpha) >= np.abs(theta):
		# 		alpha = alpha + np.sign(alpha) * 2 * np.pi
		# 	else:
		# 		theta = theta + np.sign(theta) * 2 * np.pi
		# print('after:',alpha,theta)
		# print(e,z[2,:]/(np.pi)*180,phi_t/(np.pi)*180,z_d[2,:]/(np.pi)*180,alpha/(np.pi)*180,theta/(np.pi)*180)

		return e,phi_t,alpha,theta

	def V(self,z,z_d):
		e,phi_t,alpha,theta = self.convert_to_polar(z,z_d)
		v = np.sqrt(z[2,:]**2 + z[3,:]**2)
		v_d = np.sqrt(z_d[2,:]**2 + z_d[3,:]**2)
		return 0.5 * (self.w1*alpha**2 + self.w2*theta**2 + self.w3*(v - v_d)**2)

	def dV(self,z,z_d):
		e,phi_t,alpha,theta = self.convert_to_polar(z,z_d)
		c_tmp = (self.w1*alpha + self.w2*theta) / (e + self.epsilon)
		v = np.sqrt(z[2,:]**2 + z[3,:]**2)
		v_d = np.sqrt(z_d[2,:]**2 + z_d[3,:]**2)

		return np.stack((c_tmp*np.sin(phi_t),
						 -c_tmp*np.cos(phi_t),
						 self.w1*alpha*z[3,:]/(v**2) + self.w3*(v-v_d)*z[2,:]/v,
						 -self.w1*alpha*z[2,:]/(v**2) + self.w3*(v-v_d)*z[3,:]/v),axis=0)
