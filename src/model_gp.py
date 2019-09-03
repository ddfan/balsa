import numpy as np
import matplotlib.pyplot as plt
from scaledgp import ScaledGP
from scipy import signal
from progress.bar import Bar
import random

# ALPaCA
from ALPaCA.alpaca import *
from ALPaCA.dataset import *
import time

class Model():
	def __init__(self,xdim,udim,odim,use_obs):
		self.xdim=xdim
		self.udim=udim
		self.odim=odim
		self.use_obs = use_obs
		self.verbose = True

	def predict(self,x,u,obs):
		# overload
		return None

	def train(self):
		# overload
		return None

	def add_data(self,x_next,u,x,obs,dt):
		# overload
		return None

class ModelALPaCA(Model):
	def __init__(self,xdim,udim,odim,use_obs=False):
		Model.__init__(self,xdim,udim,odim,use_obs)

		# note:  use use_obs and observations with caution.  model may overfit to this input.
		model_xdim=self.xdim + 1
		if self.use_obs:
			 model_xdim += self.odim -1
		model_ydim=self.xdim/2

		# TODO: put config someplace better (e.g., yaml file)
		self.config = {
			'meta_batch_size': 20,
			'data_horizon': 20,
			'test_horizon': 30,
			'lr': 0.01,
			'x_dim': model_xdim,
			'y_dim': model_ydim,
			'sigma_eps': [0.001, 0.001],
			'nn_layers': [128,128,64],
			'activation': 'tanh'
		}

		g = tf.Graph()
		sess = tf.Session(config=tf.ConfigProto(log_device_placement=True), graph=g)

		self.m = ALPaCA(self.config,sess,g)
		self.m.construct_model()

		self.y = np.zeros((0,model_ydim))
		self.z = np.zeros((0,model_xdim))
		self.N_data = 200
		self.N_updates = 200

		self.use_obs = False # TODO: remove this later
		self.model_trained = False

	def rotate(self,x,theta):
		x_body = np.zeros((2,1))
		x_body[0] = x[0] * np.cos(theta) + x[1] * np.sin(theta)
		x_body[1] = -x[0] * np.sin(theta) + x[1] * np.cos(theta)
		return x_body

	def make_input(self,x,mu,obs):
		# format input vector
		v = np.sqrt(x[2:3,:]**2 + x[3:4,:]**2) * x[4:5,:]
		# theta = np.arctan2(x[3]*x[4],x[2]*x[4])
		theta = obs[0]
		x_body = self.rotate(x[2:-1,:],theta)
		mu_body = self.rotate(mu,theta)
		if self.use_obs:
			z = np.concatenate((v,x_body,mu_body,obs[1:,:])).T
		else:
			z = np.concatenate((v,x_body,mu_body)).T

		return z

	def predict(self,x,mu,obs):
		# format the input and use the model to make a prediction.
		z_context = np.reshape(self.z,(1,self.z.shape[0],self.config['x_dim']))
		y_context = np.reshape(self.y,(1,self.y.shape[0],self.config['y_dim']))

		z = self.make_input(x,mu,obs)
		z = np.reshape(z,(1,1,self.config['x_dim']))

		y, var = self.m.test(z_context,y_context,z)
		y = np.squeeze(y, axis=0).T

		theta = obs[0]
		y_out = self.rotate(y,-theta)

		var = np.diag(np.squeeze(var))
		var = np.expand_dims(var,axis=0).T
		return y_out, var

	def train(self):
		# train model.  this gets called by the training thread on timer_cb() in adaptive_clbf_node.
		if self.z.shape[0] > 0:
			z_train = np.reshape(self.z,(1,self.z.shape[0],self.z.shape[1]))
			y_train = np.reshape(self.y,(1,self.y.shape[0],self.y.shape[1]))

			train_dataset = PresampledDataset(z_train,y_train)
			start_time = time.time()
			self.m.train(train_dataset, self.N_updates)
			if self.verbose:
				print('trained, elapsed time: ', time.time() - start_time)
			self.model_trained = True

	def add_data(self,x_next,x,mu_model,obs,dt):
		# add a sample to the history of data
		x_dot = (x_next[2:-1,:]-x[2:-1,:])/dt
		ynew = x_dot - mu_model
		znew = self.make_input(x,x_dot,obs)
		# theta = np.arctan2(x[3]*x[4],x[2]*x[4])
		theta=obs[0]
		ynew_rotated = self.rotate(ynew,theta)
		self.y = np.concatenate((self.y,ynew_rotated.T))
		self.z = np.concatenate((self.z,znew))

		# throw away old samples if too many samples collected.
		if self.y.shape[0] > self.N_data:
			self.y = self.y[-self.N_data:,:]
			self.z = self.z[-self.N_data:,:]
			# self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
			# self.z = np.delete(self.z,random.randint(0,self.N_data-1),axis=0)

		if self.verbose:
			print("obs", obs)
			print("y_out",y_out)
			print("ynew",ynew)
			print("znew",znew)
			print("x_dot",x_dot)
			print("mu_model",mu_model)
			print("dt",dt)
			print("n data:", self.y.shape[0])
			print("prediction error:", self.predict_error)
			print("predict var:", self.predict_var)

class ModelGP(Model):
	def __init__(self,xdim,udim,odim,use_obs=False):
		Model.__init__(self,xdim,udim,odim,use_obs)
		# note:  use use_obs and observations with caution.  model may overfit to this input.
		model_xdim=self.xdim + 1
		if self.use_obs:
			 model_xdim += self.odim -1
		model_ydim=self.xdim/2

		self.m = ScaledGP(xdim=model_xdim,ydim=model_ydim)
		self.y = np.zeros((0,model_ydim))
		self.z = np.zeros((0,model_xdim))
		self.N_data = 400

		self.model_trained = False

	def rotate(self,x,theta):
		x_body = np.zeros((2,1))
		x_body[0] = x[0] * np.cos(theta) + x[1] * np.sin(theta)
		x_body[1] = -x[0] * np.sin(theta) + x[1] * np.cos(theta)
		return x_body

	def make_input(self,x,mu,obs):
		# format input vector
		v = np.sqrt(x[2:3,:]**2 + x[3:4,:]**2) * x[4:5,:]
		# theta = np.arctan2(x[3]*x[4],x[2]*x[4])
		theta = obs[0]
		x_body = self.rotate(x[2:-1,:],theta)
		mu_body = self.rotate(mu,theta)
		if self.use_obs:
			z = np.concatenate((v,x_body,mu_body,obs[1:,:])).T
		else:
			z = np.concatenate((v,x_body,mu_body)).T

		return z

	def predict(self,x,mu,obs):
		# format the input and use the model to make a prediction.
		z = self.make_input(x,mu,obs)
		y, var = self.m.predict(z)
		# theta = np.arctan2(x[3]*x[4],x[2]*x[4])
		theta=obs[0]
		y_out = self.rotate(y.T,-theta)
		return y_out, var.T

	def train(self):
		# train model.  this gets called by the training thread on timer_cb() in adaptive_clbf_node.
		if self.z.shape[0] > 0:
			self.m.optimize(self.z,self.y)
			self.model_trained = True

	def add_data(self,x_next,x,mu_model,obs,dt):
		# add a sample to the history of data
		x_dot = (x_next[2:-1,:]-x[2:-1,:])/dt
		ynew = x_dot - mu_model
		znew = self.make_input(x,x_dot,obs)
		# theta = np.arctan2(x[3]*x[4],x[2]*x[4])
		theta=obs[0]
		ynew_rotated = self.rotate(ynew,theta)
		self.y = np.concatenate((self.y,ynew_rotated.T))
		self.z = np.concatenate((self.z,znew))

		# throw away old samples if too many samples collected.
		if self.y.shape[0] > self.N_data:
			self.y = self.y[-self.N_data:,:]
			self.z = self.z[-self.N_data:,:]
			# self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
			# self.z = np.delete(self.z,random.randint(0,self.N_data-1),axis=0)

		if self.verbose:
			print("obs", obs)
			print("y_out",y_out)
			print("ynew",ynew)
			print("znew",znew)
			print("x_dot",x_dot)
			print("mu_model",mu_model)
			print("dt",dt)
			print("n data:", self.y.shape[0])
			print("prediction error:", self.predict_error)
			print("predict var:", self.predict_var)
