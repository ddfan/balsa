import numpy as np
import matplotlib.pyplot as plt

class Dynamics():
	def __init__(self,xdim,udim):
		self.xdim=xdim
		self.udim=udim

	def f(self,x):
		# overload
		return None

	def g(self,x):
		# overload
		return None

	def pseudoinv(self,x):
		return np.matmul(np.linalg.inv(np.matmul(x.T,x)),x.T)

	def convert_z_to_x(self,z):
		v=np.sqrt(z[2,:]**2 + z[3,:]**2) * z[4,:]
		return np.stack((z[0,:],z[1,:],np.arctan2(z[4,:]*z[3,:],z[4,:]*z[2,:]),v),axis=0)

	def convert_x_to_z(self,x):
		v_sign = 1.0
		if x[3,:] < 0:
			v_sign = -1
		return np.stack((x[0,:],x[1,:],x[3,:]*np.cos(x[2,:]),x[3,:]*np.sin(x[2,:]),np.array([v_sign])),axis=0)

class DynamicsAckermann(Dynamics):
	def __init__(self,xdim=4,udim=2):
		Dynamics.__init__(self,xdim=xdim,udim=udim)

	def f(self,x):
		return np.stack((x[3,:] * np.cos(x[2,:]), x[3,:] * np.sin(x[2,:]), np.array([0]), np.array([0])))

	def g(self,x):
		return np.stack((np.array([0,0]),np.array([0,0]),np.append(x[3,:],0),np.array([0,1])))

class DynamicsAckermannModified(Dynamics):
	def __init__(self,xdim=4,udim=2):
		Dynamics.__init__(self,xdim=xdim,udim=udim)

	def f(self,x):
		v_modified = x[3,:]**2 / (2*np.tanh(x[3,:]))
		return np.stack((v_modified * np.cos(x[2,:]), v_modified * np.sin(x[2,:]), x[3,:], -0.5*x[3,:]))

	def g(self,x):
		return np.stack((np.array([0,0]),np.array([0,0]),np.append(x[3,:],0),np.array([0,1])))*1.2


class DynamicsAckermannZ(Dynamics):
	def __init__(self,xdim=4,udim=2,epsilon=1e-6):
		self.epsilon = epsilon
		Dynamics.__init__(self,xdim=xdim,udim=udim)

	def f(self,z):
		return np.stack((np.array([0]),np.array([0]))) * z[4,:]

	def g(self,z):
		v=(np.sqrt(z[2,:]**2 + z[3,:]**2) + self.epsilon) * z[4,:]
		return np.stack((np.concatenate((-z[3,:]*v,z[2,:]/v)),np.concatenate((z[2,:]*v,z[3,:]/v))))


class DynamicsAckermannZModified(Dynamics):
	def __init__(self,xdim=4,udim=2,epsilon=1e-6,disturbance_scale_pos = 0.5,disturbance_scale_vel = 2.0,control_input_scale = 2.0):
		self.epsilon = epsilon
		Dynamics.__init__(self,xdim=xdim,udim=udim)

		self.disturbance_scale_pos = disturbance_scale_pos
		self.disturbance_scale_vel = disturbance_scale_vel
		self.control_input_scale = control_input_scale
		
	def f(self,z):
		v=np.sqrt(z[2,:]**2 + z[3,:]**2) * z[4,:]
		theta = np.arctan2(z[3]*z[4],z[2]*z[4])
		v_disturbance_body = [np.tanh(v**2)*self.disturbance_scale_vel, (0.1+v)*self.disturbance_scale_vel]
		v_disturbance_world = [v_disturbance_body[0] * np.cos(theta) - v_disturbance_body[1] * np.sin(theta),
							   v_disturbance_body[0] * np.sin(theta) + v_disturbance_body[1] * np.cos(theta)]
		return np.array([v_disturbance_world[0], v_disturbance_world[1]])
		
	def g(self,z):
		v=(np.sqrt(z[2,:]**2 + z[3,:]**2)  + self.epsilon) * z[4,:]
		return self.control_input_scale * np.stack((np.concatenate((-z[3,:]*v,z[2,:]/v)),np.concatenate((z[2,:]*v,z[3,:]/v))))

	def step(self,z,u,dt):
		v=np.sqrt(z[2,:]**2 + z[3,:]**2)
		znext = np.zeros((self.xdim+1,1))
		znext[0:2,:] = z[0:2,:] + dt*(z[2:-1,:] + np.array([np.sin(v**3)*self.disturbance_scale_pos, np.cos(-v)*self.disturbance_scale_pos]))
		znext[2:-1,:] = z[2:-1,:] + dt*(self.f(z) + np.matmul(self.g(z),u))
		znext[-1] = z[-1]
		return znext
