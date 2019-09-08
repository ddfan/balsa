#! /usr/bin/env python
import os
import rospy
import actionlib
import numpy as np

from dynamic_reconfigure.client import Client as DynamicReconfigureClient

import controller_adaptiveclbf.msg
from controller_adaptiveclbf.srv import *

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

# Vannila Netowrk
from vanilla_nn import VanillaNN

BASE_PATH = os.path.expanduser('~/Documents')

class ModelService(object):
	_train_result = controller_adaptiveclbf.msg.TrainModelResult()

	def __init__(self,xdim,odim,use_obs, use_service = True):

		self.xdim=xdim
		self.odim=odim
		self.use_obs = use_obs
		self.verbose = True

		self.config = {}

		if use_service:
			# train action server
			self._action_service = actionlib.SimpleActionServer('train_model_service', controller_adaptiveclbf.msg.TrainModelAction, execute_cb=self.train, auto_start = False)
			self._action_service.start()
			# add data service
			self._add_data_srv = rospy.Service('add_data_2_model', AddData2Model, self.add_data)
			# predict service
			self._predict_srv = rospy.Service('predict_model', PredictModel, self.predict)

			# Create a dynamic reconfigure client
			self.dyn_reconfig_client = DynamicReconfigureClient('controller_adaptiveclbf_reconfig', timeout=30, config_callback=self.reconfigure_cb)

			self.N_data = rospy.get_param('/controller_adaptiveclbf/N_data',200)
			self.verbose = rospy.get_param('/controller_adaptiveclbf/learning_verbose',False)


	def reconfigure_cb(self, config):
		self.N_data = config["N_data"]
		self.verbose = config["learning_verbose"]
		self.N_updates = config["N_updates"]
		self.config["meta_batch_size"] = config["meta_batch_size"]
		self.config["data_horizon"] = config["data_horizon"]
		self.config["test_horizon"] = config["test_horizon"]
		self.config["learning_rate"] = config["learning_rate"]
		self.config["min_datapoints"] = config["min_datapoints"]
		self.config["save_data_interval"] = config["save_data_interval"]

	def predict(self,req):
		# overload
		return None

	def train(self,goal):
		# overload
		return None

	def add_data(self,req):
		# overload
		return None


	def scale_data(self,x,xmean,xstd):
		if (xstd == 0).any():
			return (x-xmean)
		else:
			return (x - xmean) / xstd

	def unscale_data(self,x,xmean,xstd):
		if (xstd == 0).any():
			return x + xmean
		else:
			return x * xstd + xmean

class ModelVanillaService(ModelService):
	def __init__(self,xdim,odim,use_obs=False,use_service=True):
		ModelService.__init__(self,xdim,odim,use_obs,use_service)
		# note:  use use_obs and observations with caution.  model may overfit to this input.
		model_xdim=self.xdim/2
		if self.use_obs:
			 model_xdim += self.odim
		model_ydim=self.xdim/2

		self.Zmean = np.zeros((1,model_xdim))
		self.Zstd = np.ones((1,model_xdim))
		self.ymean = np.zeros((1,model_ydim))
		self.ystd = np.ones((1,model_ydim))
		self.model_trained = False

		self.y = np.zeros((0,model_ydim))
		self.Z = np.zeros((0,model_xdim))

		self.m = VanillaNN(model_xdim, model_ydim)


	def rotate(self,x,theta):
		x_body = np.zeros((2,1))
		x_body[0] = x[0] * np.cos(theta) + x[1] * np.sin(theta)
		x_body[1] = -x[0] * np.sin(theta) + x[1] * np.cos(theta)
		return x_body

	def make_input(self,x,obs):
		# format input vector
		theta = obs[0]
		x_body = self.rotate(x[2:-1,:],theta)
		if self.use_obs:
			Z = np.concatenate((x_body,obs[1:,:])).T
		else:
			Z = np.concatenate((x_body)).T

		Z = self.scale_data(Z,self.Zmean,self.Zstd)
		return Z

	def predict(self,req):
		if not hasattr(self, 'Z'):
			resp = PredictModelResponse()
			resp.result = False
			return resp
		x = np.expand_dims(req.x, axis=0).T
		obs = np.expand_dims(req.obs, axis=0).T

		Z = self.make_input(x,obs)

		# needed fix for weird shapes of tensors
		ZN = np.concatenate((self.Z[-9:,:],Z))
		Y = self.y[-10:,:]
		# print("predict ZN.shape", ZN.shape)
		# print("predict Y.shape", Y.shape)

		y, var, _ = self.m.predict(ZN,Y)

		theta = obs[0]
		y_out = self.rotate(y,-theta).T

		resp = PredictModelResponse()
		resp.y_out = y_out.flatten()
		resp.var = var
		resp.result = True

		return resp

	def train(self, goal=None):
		success = True

		if goal is not None:
			# goal was cancelled
			if self._action_service.is_preempt_requested():
				print("Preempt training request")
				self._action_service.set_preempted()
				success = False

		# train model.  this gets called by the training thread on timer_cb() in adaptive_clbf_node.
		# print("train z then y shapes: ", self.Z.shape, self.y.shape)
		if success and self.Z.shape[0] > 0 and self.Z.shape[0] == self.y.shape[0]:
			self.m.train(self.Z, self.y)
			if goal is not None:
				self._train_result.model_trained = True
				self._action_service.set_succeeded(self._train_result)
		else:
			if goal is not None:
				self._train_result.model_trained = False
				self._action_service.set_succeeded(self._train_result)

	def add_data(self,req):
		if not hasattr(self, 'y'):
			return AddData2ModelResponse(False)

		x_next = np.expand_dims(req.x_next, axis=0).T
		x = np.expand_dims(req.x, axis=0).T
		mu_model = np.expand_dims(req.mu_model, axis=0).T
		obs = np.expand_dims(req.obs, axis=0).T
		dt = req.dt

		# add a sample to the history of data
		x_dot = (x_next[2:-1,:]-x[2:-1,:])/dt
		ynew = x_dot - mu_model
		Znew = self.make_input(x,obs)

		theta=obs[0]
		ynew_rotated = self.rotate(ynew,theta)
		self.y = np.concatenate((self.y,ynew_rotated.T))
		self.Z = np.concatenate((self.Z,Znew))

		# throw away old samples if too many samples collected.
		if self.y.shape[0] > self.N_data:
			self.y = self.y[-self.N_data:,:]
			self.Z = self.Z[-self.N_data:,:]
			# self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
			# self.Z = np.delete(self.Z,random.randint(0,self.N_data-1),axis=0)

		if self.verbose:
			print("obs", obs)
			print("ynew",ynew)
			print("ynew_rotated", ynew_rotated)
			print("Znew",Znew)
			print("x_dot",x_dot)
			print("mu_model",mu_model)
			print("dt",dt)
			print("n data:", self.y.shape[0])

		return AddData2ModelResponse(True)

class ModelALPaCAService(ModelService):
	def __init__(self,xdim,odim,use_obs=False,use_service=True):
		ModelService.__init__(self,xdim,odim,use_obs,use_service)
		# note:  use use_obs and observations with caution.  model may overfit to this input.
		model_xdim=self.xdim/2
		if self.use_obs:
			 model_xdim += self.odim
		model_ydim=self.xdim/2

		self.Zmean = np.zeros((1,model_xdim))
		self.Zstd = np.ones((1,model_xdim))
		self.ymean = np.zeros((1,model_ydim))
		self.ystd = np.ones((1,model_ydim))
		self.model_trained = False

		# TODO: put config someplace better (e.g., yaml file)
		self.config = {
			'meta_batch_size': 64,
			'data_horizon': 20,
			'test_horizon': 20,
			# 'meta_batch_size': 100,
			# 'data_horizon': 100,
			# 'test_horizon': 50,
			'lr': 0.005,
			'x_dim': model_xdim,
			'y_dim': model_ydim,
			'sigma_eps': [1.0, 1.0],
			'nn_layers': [128,128,128],
			'activation': 'tanh',
			'min_datapoints': 1000,
			'save_data_interval': 1000
		}

		g = tf.Graph()
		config = tf.ConfigProto(log_device_placement=True)
		config.gpu_options.allow_growth=True
		sess = tf.Session(config=config, graph=g)

		self.m = ALPaCA(self.config,sess,g)
		self.m.construct_model()

		self.y = np.zeros((0,model_ydim))
		self.Z = np.zeros((0,model_xdim))
		self.N_data = 10000
		self.N_updates = 50

	def rotate(self,x,theta):
		x_body = np.zeros((2,1))
		x_body[0] = x[0] * np.cos(theta) + x[1] * np.sin(theta)
		x_body[1] = -x[0] * np.sin(theta) + x[1] * np.cos(theta)
		return x_body

	def make_input(self,x,obs):
		# format input vector
		theta = obs[0]
		x_body = self.rotate(x[2:-1,:],theta)
		if self.use_obs:
			Z = np.concatenate((x_body,obs[1:,:])).T
		else:
			Z = np.concatenate((x_body)).T

		Z = self.scale_data(Z,self.Zmean,self.Zstd)
		return Z

	def predict(self,req):
		if not hasattr(self, 'Z'):
			resp = PredictModelResponse()
			resp.result = False
			return resp
		x = np.expand_dims(req.x, axis=0).T
		obs = np.expand_dims(req.obs, axis=0).T

		# format the input and use the model to make a prediction.
		z_context = np.reshape(self.Z[-self.config["data_horizon"]:,:],(1,np.minimum(self.config["data_horizon"],self.Z.shape[0]),self.config['x_dim']))
		y_context = np.reshape(self.y[-self.config["data_horizon"]:,:],(1,np.minimum(self.config["data_horizon"],self.y.shape[0]),self.config['y_dim']))

		Z = self.make_input(x,obs)
		Z = np.reshape(Z,(1,1,self.config['x_dim']))

		y, var = self.m.test(z_context,y_context,Z)
		y = np.squeeze(y, axis=0).T

		theta = obs[0]
		y_out = self.rotate(y,-theta).T
		# y_out = self.unscale_data(y_out,self.ymean,self.ystd)

		var = np.diag(np.squeeze(var))
		# var = np.expand_dims(var,axis=0).T
		resp = PredictModelResponse()
		resp.y_out = y_out.flatten()
		resp.var = var
		resp.result = True
		return resp

	def train(self,goal=None):
		success = True

		if goal is not None:
			# goal was cancelled
			if self._action_service.is_preempt_requested():
				print("Preempt training request")
				self._action_service.set_preempted()
				success = False

		if not hasattr(self, 'Z'):
			success = False

		# train model.  this gets called by the training thread on timer_cb() in adaptive_clbf_node.
		if success and self.Z.shape[0] > self.config["min_datapoints"] and self.Z.shape[0] == self.y.shape[0]:
			if not self.model_trained:
				self.Zmean = np.mean(self.Z,axis=0)
				self.Zstd = np.std(self.Z,axis=0)
				self.Z = self.scale_data(self.Z,self.Zmean, self.Zstd)
				self.ymean = np.mean(self.y,axis=0)
				self.ystd = np.std(self.y,axis=0)
				self.y = self.scale_data(self.y,self.ymean, self.ystd)
				self.model_trained = True
				print("Mean and std of data computed with # data points:", self.Z.shape[0])
				print("Zmean:", self.Zmean)
				print("Zstd:", self.Zstd)
				print("ymean:", self.ymean)
				print("ystd:", self.ystd)


			z_train = np.reshape(self.Z,(1,self.Z.shape[0],self.Z.shape[1]))
			y_train = np.reshape(self.y,(1,self.y.shape[0],self.y.shape[1]))
			train_dataset = PresampledDataset(z_train,y_train)
			# start_time = time.time()
			self.m.train(train_dataset, self.N_updates, self.config)
			# print('trained, elapsed time: ', time.time() - start_time)
			if goal is not None:
				self._train_result.model_trained = True
				self._action_service.set_succeeded(self._train_result)
		else:
			if goal is not None:
				self._train_result.model_trained = False
				self._action_service.set_succeeded(self._train_result)

	def add_data(self,req):
		if not hasattr(self, 'y'):
			return AddData2ModelResponse(False)

		x_next = np.expand_dims(req.x_next, axis=0).T
		x = np.expand_dims(req.x, axis=0).T
		mu_model = np.expand_dims(req.mu_model, axis=0).T
		obs = np.expand_dims(req.obs, axis=0).T
		dt = req.dt

		# add a sample to the history of data
		x_dot = (x_next[2:-1,:]-x[2:-1,:])/dt
		ynew = x_dot - mu_model
		Znew = self.make_input(x,obs)
		# theta = np.arctan2(x[3]*x[4],x[2]*x[4])
		theta=obs[0]
		ynew_rotated = self.rotate(ynew,theta).T
		# ynew_rotated = self.scale_data(ynew_rotated,self.ymean,self.ystd)
		self.y = np.concatenate((self.y,ynew_rotated))
		self.Z = np.concatenate((self.Z,Znew))

		# throw away old samples if too many samples collected.
		if self.y.shape[0] > self.N_data:
			self.y = self.y[-self.N_data:,:]
			self.Z = self.Z[-self.N_data:,:]
			# self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
			# self.Z = np.delete(self.Z,random.randint(0,self.N_data-1),axis=0)

		if self.y.shape[0] % self.config["save_data_interval"] == 1:
			print("Saving data to " + BASE_PATH + "...")
			np.save(os.path.join(BASE_PATH, "data_input"),self.Z)
			np.save(os.path.join(BASE_PATH, "data_output"),self.y)
			print("Data saved!")

		if self.verbose:
			print("obs", obs)
			# print("y_out",y_out)
			print("ynew",ynew)
			print("Znew",Znew)
			print("x_dot",x_dot)
			print("mu_model",mu_model)
			print("dt",dt)
			print("n data:", self.y.shape[0])
			# print("prediction error:", self.predict_error)
			# print("predict var:", self.predict_var)

		return AddData2ModelResponse(True)

class ModelGPService(ModelService):
	def __init__(self,xdim,odim,use_obs=False,use_service=True):
		ModelService.__init__(self,xdim,odim,use_obs,use_service)
		# note:  use use_obs and observations with caution.  model may overfit to this input.
		model_xdim=self.xdim/2
		if self.use_obs:
			 model_xdim += self.odim
		model_ydim=self.xdim/2

		self.m = ScaledGP(xdim=model_xdim,ydim=model_ydim)
		self.y = np.zeros((0,model_ydim))
		self.Z = np.zeros((0,model_xdim))
		self.N_data = 400

	def rotate(self,x,theta):
		x_body = np.zeros((2,1))
		x_body[0] = x[0] * np.cos(theta) + x[1] * np.sin(theta)
		x_body[1] = -x[0] * np.sin(theta) + x[1] * np.cos(theta)
		return x_body

	def make_input(self,x,obs):
		# format input vector
		theta = obs[0]
		x_body = self.rotate(x[2:-1,:],theta)
		if self.use_obs:
			Z = np.concatenate((x_body,obs[1:,:])).T
		else:
			Z = np.concatenate((x_body)).T

		#normalize input by mean and variance
		# Z = (Z - self.Zmean) / self.Zvar

		return Z

	def predict(self,req):
		if not hasattr(self, 'm'):
			resp = PredictModelResponse()
			resp.result = False
			return resp
		x = np.expand_dims(req.x, axis=0).T
		obs = np.expand_dims(req.obs, axis=0).T

		# format the input and use the model to make a prediction.
		Z = self.make_input(x,obs)
		y, var = self.m.predict(Z)
		# theta = np.arctan2(x[3]*x[4],x[2]*x[4])
		theta=obs[0]
		y_out = self.rotate(y.T,-theta)

		resp = PredictModelResponse()
		resp.y_out = y_out.flatten()
		resp.var = var.T.flatten()
		resp.result = True

		return resp

	def train(self, goal=None):
		success = True

		if goal is not None:
			# goal was cancelled
			if self._action_service.is_preempt_requested():
				print("Preempt training request")
				self._action_service.set_preempted()
				success = False

		# train model.  this gets called by the training thread on timer_cb() in adaptive_clbf_node.
		if success and self.Z.shape[0] > 0 and self.Z.shape[0] == self.y.shape[0]:
			self.m.optimize(self.Z,self.y)
			if goal is not None:
				self._train_result.model_trained = True
				self._action_service.set_succeeded(self._train_result)

	def add_data(self,req):
		if not hasattr(self, 'y'):
			return AddData2ModelResponse(False)

		x_next = np.expand_dims(req.x_next, axis=0).T
		x = np.expand_dims(req.x, axis=0).T
		mu_model = np.expand_dims(req.mu_model, axis=0).T
		obs = np.expand_dims(req.obs, axis=0).T
		dt = req.dt

		# add a sample to the history of data
		x_dot = (x_next[2:-1,:]-x[2:-1,:])/dt
		ynew = x_dot - mu_model
		Znew = self.make_input(x,obs)
		# theta = np.arctan2(x[3]*x[4],x[2]*x[4])
		theta=obs[0]
		ynew_rotated = self.rotate(ynew,theta)
		self.y = np.concatenate((self.y,ynew_rotated.T))
		self.Z = np.concatenate((self.Z,Znew))

		# throw away old samples if too many samples collected.
		if self.y.shape[0] > self.N_data:
			self.y = self.y[-self.N_data:,:]
			self.Z = self.Z[-self.N_data:,:]
			# self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
			# self.Z = np.delete(self.Z,random.randint(0,self.N_data-1),axis=0)

		if self.verbose:
			print("obs", obs)
			print("ynew",ynew)
			print("ynew_rotated", ynew_rotated)
			print("Znew",Znew)
			print("x_dot",x_dot)
			print("mu_model",mu_model)
			print("dt",dt)
			print("n data:", self.y.shape[0])
			# print("prediction error:", self.predict_error)
			# print("predict var:", self.predict_var)

		return AddData2ModelResponse(True)

if __name__ == '__main__':
    rospy.init_node('model_service')
    server = ModelVanillaService(4,6, use_obs = True) # TODO: put this in yaml or somewhere else
    # server = ModelALPaCAService(4,6, use_obs = True) # TODO: put this in yaml or somewhere else
    # server = ModelGPService(4,6, use_obs = True) # TODO: put this in yaml or somewhere else
    rospy.spin()
