import numpy as np
import copy

import rospy
import actionlib
import controller_adaptiveclbf.msg
from controller_adaptiveclbf.srv import *

from qp_solver import QPSolve
from dynamics import DynamicsAckermannZ
from cbf import BarrierAckermannVelocityZ, BarrierAckermannPointZ
from lyapunov import LyapunovAckermannZ
from model_service import ModelVanillaService, ModelALPaCAService, ModelGPService

class AdaptiveClbf(object):
	def __init__(self,odim=2, use_service = True, ):
		self.xdim = 4
		self.udim = 2
		self.odim = odim
		self.use_service = use_service

		self.u_lim = np.array([[-2.0,2.0],[-1.0,1.0]])
		self.K=np.block([[np.zeros((2,2)), np.zeros((2,2))],[np.eye(2), np.eye(2)]])

		self.dyn = DynamicsAckermannZ()

		self.model_trained = False
		
		if self.use_service:
			## train model action
			self.train_model_action_client = actionlib.SimpleActionClient('train_model_service', controller_adaptiveclbf.msg.TrainModelAction)
			self.train_model_action_client.wait_for_server()
			self.train_model_goal = controller_adaptiveclbf.msg.TrainModelGoal()

			## add data srv
			rospy.wait_for_service('add_data_2_model')
			self.model_add_data_srv = rospy.ServiceProxy('add_data_2_model', AddData2Model)

			## predict srv
			rospy.wait_for_service('predict_model')
			self.model_predict_srv = rospy.ServiceProxy('predict_model', PredictModel)
		else:
			# setup non-service model object
			# self.model = ModelVanillaService(self.xdim,self.odim,use_obs=True,use_service=False)
			# self.model = ModelGPService(self.xdim,self.odim,use_obs=True,use_service=False)
			self.model = ModelALPaCAService(self.xdim,self.odim,use_obs=True,use_service=False)

		self.clf = LyapunovAckermannZ(w1=10.0,w2=1.0,w3=1.0,epsilon=1.0)
		self.qpsolve = QPSolve(dyn=self.dyn,cbf_list=[],clf=self.clf,u_lim=self.u_lim,u_cost=0.0,u_prev_cost=1.0,p1_cost=1.0e8,p2_cost=1.0e8,verbose=False)

		x_init = np.zeros((self.xdim,1))
		x_init[3] = 0.01
		self.z_ref = self.dyn.convert_x_to_z(x_init)
		self.z = copy.copy(self.z_ref)
		self.z_ref_dot = np.zeros((self.xdim,1))
		self.z_dot = np.zeros((self.xdim/2,1))
		self.z_prev = copy.copy(self.z)
		self.y_out = np.zeros((self.xdim/2,1))
		self.mu_prev = np.zeros((self.xdim,1))
		self.u_prev = np.zeros((self.udim,1))
		self.u_prev_prev = np.zeros((self.udim,1))
		self.obs_prev = np.zeros((self.odim,1))
		self.dt = 0.1
		self.max_error = 1.0

		self.barrier_locations={}
		self.barrier_locations["x"]=np.array([])
		self.barrier_locations["y"]=np.array([])
		self.barrier_radius = 1.0

		self.measurement_noise = 1.0
		self.true_dyn = None
		self.true_predict_error = 0.0
		self.predict_error = 0
		self.predict_var = np.zeros((self.xdim/2,1))

		self.debug={}

	def wrap_angle(self,a):
		return (a + np.pi) % (2 * np.pi) - np.pi

	def saturate(self,u,ulim):
		return np.array([np.clip(u[0],ulim[0][0],ulim[0][1]), np.clip(u[1],ulim[1][0],ulim[1][1])])

	def update_params(self,params):
		self.params = params

		self.vehicle_length = self.params["vehicle_length"]
		self.steering_limit = self.params["steering_limit"]
		self.max_accel = self.params["max_accel"]
		self.min_accel = self.params["min_accel"]
		self.u_lim = np.array([[np.tan(-self.steering_limit)/self.vehicle_length,np.tan(self.steering_limit)/self.vehicle_length],
								[self.min_accel,self.max_accel]])
		self.qpsolve.u_lim = self.u_lim

		self.k1 = self.params["kp_z"]
		self.k2 = self.params["kd_z"]
		self.A=np.block([[np.zeros((2,2)), np.eye(2)],[-self.k1*np.eye(2), -self.k2*np.eye(2)]])
		self.qpsolve.update_ricatti(self.A)
		self.K=np.block([[self.k1*np.eye(2), self.k2*np.eye(2)]])
		self.max_error = self.params["max_error"]

		self.clf.epsilon = self.params["clf_epsilon"]
		self.measurement_noise = self.params["measurement_noise"]

		self.qpsolve.u_cost = self.params["qp_u_cost"]
		self.qpsolve.u_prev_cost = self.params["qp_u_prev_cost"]
		self.qpsolve.p1_cost = self.params["qp_p1_cost"]
		self.qpsolve.p2_cost = self.params["qp_p2_cost"]
		self.qpsolve.verbose = self.params["qp_verbose"]
		self.qpsolve.ksig = self.params["qp_ksig"]
		self.qpsolve.max_var = self.params["qp_max_var"]
		self.verbose = self.params["verbose"]

		self.dt = self.params["dt"]

		# update model params if not using service calls
		if not self.use_service:
			self.model.N_data = self.params["N_data"]
			self.model.verbose = self.params["learning_verbose"]
			self.model.N_updates = self.params["N_updates"]
			self.model.config["meta_batch_size"] = self.params["meta_batch_size"]
			self.model.config["data_horizon"] = self.params["data_horizon"]
			self.model.config["test_horizon"] = self.params["test_horizon"]
			self.model.config["learning_rate"] = self.params["learning_rate"]
			self.model.config["min_datapoints"] = self.params["min_datapoints"]
			self.model.config["save_data_interval"] = self.params["save_data_interval"]

	def update_barrier_locations(self,x,y,radius):
		self.barrier_locations["x"] = x
		self.barrier_locations["y"] = y
		self.barrier_radius = radius

	def update_barriers(self):
		cbf_list = []
		bar_loc_x = copy.copy(self.barrier_locations["x"])
		bar_loc_y = copy.copy(self.barrier_locations["y"])
		bar_rad = self.barrier_radius

		if self.params["use_barrier_vel"]:
			cbf_list = cbf_list + \
							[BarrierAckermannVelocityZ(bound_from_above=True, v_lim = self.params["max_velocity"], gamma=self.params["barrier_vel_gamma"]),
							BarrierAckermannVelocityZ(bound_from_above=False, v_lim = self.params["min_velocity"], gamma=self.params["barrier_vel_gamma"])]

		if self.params["use_barrier_pointcloud"]:
			cbf_list = cbf_list + \
							[BarrierAckermannPointZ(x=bar_loc_x[i],y=bar_loc_y[i], radius=bar_rad, gamma_p=self.params["barrier_pc_gamma_p"], gamma=self.params["barrier_pc_gamma"]) for i in range(bar_loc_x.size)]
		self.qpsolve.cbf_list = cbf_list

	def get_control(self,z,z_ref,z_ref_dot,dt,obs=None,train=False,use_model=False,add_data=True,check_model=True,use_qp=True):
		assert z.shape[0] == self.xdim + 1
		assert z_ref_dot.shape[0] == self.xdim + 1
		assert z_ref.shape[0] == self.xdim + 1

		self.update_barriers()

		self.z = copy.copy(z)
		self.z_ref = copy.copy(z_ref)
		self.obs = copy.copy(obs)

		mu_ad = np.zeros((self.xdim/2,1))
		mDelta = np.zeros((self.xdim/2,1))
		sigDelta = np.zeros((self.xdim/2,1))
		rho = np.zeros((self.xdim/2,1))
		trueDelta = np.zeros((self.xdim/2,1))

		e = self.z_ref[:-1,:]-self.z[:-1,:]
		mu_pd = np.matmul(self.K,e)
		mu_pd = np.clip(mu_pd,-self.max_error,self.max_error)

		# self.z_ref_dot = (self.z_ref_next[:-1,:] - self.z_ref[:-1,:]) / dt # use / self.dt for test_adaptive_clbf
		self.z_ref_dot = copy.copy(z_ref_dot)
		mu_rm = self.z_ref_dot[2:-1]

		mu_model = np.matmul(self.dyn.g(self.z_prev),self.u_prev) + self.dyn.f(self.z_prev)
		mu_model = np.clip(mu_model,-self.max_error,self.max_error)

		if add_data:
			if self.use_service:
				try:
					self.model_add_data_srv(self.z.flatten(),self.z_prev.flatten(),mu_model.flatten(),self.obs_prev.flatten(),dt)
				except:
					print("add data service unavailable")
			else:
				req = AddData2Model()
				req.x_next = self.z.flatten()
				req.x = self.z_prev.flatten()
				req.mu_model = mu_model.flatten()
				req.obs = self.obs_prev.flatten()
				req.dt = dt
				self.model.add_data(req)

		self.z_dot = (self.z[2:-1,:]-self.z_prev[2:-1,:])/dt - mu_model

		# if check_model and self.model.model_trained:
		if check_model and self.model_trained:
			# check how the model is doing.  compare the model's prediction with the actual sampled data.
			predict_service_success = False
			result = None
			if self.use_service:
				try:
					result = self.model_predict_srv(self.z_prev.flatten(),self.obs_prev.flatten())
					if result.result:
						predict_service_success = True
				except:
					print("predict service unavailable")
			else:
				req = PredictModel()
				req.x = self.z_prev.flatten()
				req.obs = self.obs_prev.flatten()
				result = self.model.predict(req)
				predict_service_success = True

			if predict_service_success:
				self.y_out = np.expand_dims(result.y_out, axis=0).T
				var = np.expand_dims(result.var, axis=0).T

				if self.verbose:
					print("predicted y_out: ", self.y_out)
					print("predicted ynew: ", self.z_dot)
					print("predicted var: ", var)

				self.predict_error = np.linalg.norm(self.y_out - self.z_dot)
				self.predict_var = var

		if use_model and self.model_trained:
			predict_service_success = False
			result = None
			if self.use_service:
				try:
					result = self.model_predict_srv(self.z.flatten(),self.obs.flatten())
					if result.result:
						predict_service_success = True
				except:
					print("predict service unavailable")
			else:
				req = PredictModel()
				req.x = self.z.flatten()
				req.obs = self.obs.flatten()
				result = self.model.predict(req)
				predict_service_success = True

			if predict_service_success:
				mDelta = np.expand_dims(result.y_out, axis=0).T
				sigDelta = np.expand_dims(result.var, axis=0).T

				# log error if true system model is available
				if self.true_dyn is not None:
					trueDelta = self.true_dyn.f(self.z) - self.dyn.f(self.z)
					self.true_predict_error = np.linalg.norm(trueDelta - mDelta)

				# rho = self.measurement_noise / (self.measurement_noise + (sigDelta - 1.0) + 1e-6)
				rho = self.measurement_noise / (self.measurement_noise + (sigDelta) + 1e-6)
				# rho = sigDelta * self.measurement_noise
				mu_ad = mDelta * rho
				# sigDelta = (sigDelta - 1.0) / self.measurement_noise 

		mu_d = mu_rm + mu_pd - mu_ad
		self.mu_qp = np.zeros((self.xdim/2,1))
		if use_qp:
			self.mu_qp = self.qpsolve.solve(self.z,self.z_ref,mu_d,sigDelta)

		self.mu_new = mu_d + self.mu_qp
		self.u_new = np.matmul(np.linalg.inv(self.dyn.g(self.z)), (self.mu_new-self.dyn.f(self.z)))

		u_new_unsaturated = copy.copy(self.u_new)
		#saturate in case constraints are not met
		self.u_new = self.saturate(self.u_new,self.u_lim)

		if self.verbose:
			print('z: ', self.z.T)
			print('z_ref: ', self.z_ref.T)
			print('mu_rm', mu_rm)
			print('mu_pd', mu_pd)
			print('mu_ad', mu_ad)
			print('mu_d', mu_d)
			print('mu_model', mu_model)
			print('rho', rho)
			print('mu_qp', self.mu_qp)
			print('mu',self.mu_new)
			print('u_new', self.u_new)
			print('u_unsat', u_new_unsaturated)
			print('trueDelta',trueDelta)
			print('true predict error', self.true_predict_error)
			print('mDelta', mDelta)
			print('sigDelta', sigDelta)

		self.debug["z"] = self.z.flatten().tolist()
		self.debug["z_ref"] = self.z_ref.flatten().tolist()
		self.debug["z_dot"] = self.z_dot.flatten().tolist()
		self.debug["y_out"] = self.y_out.flatten().tolist()
		self.debug["mu_rm"] = self.z_ref_dot.flatten().tolist()
		self.debug["mu_pd"] = mu_pd.flatten().tolist()
		self.debug["mu_ad"] = mu_ad.flatten().tolist()
		self.debug["mu_model"] = mu_model.flatten().tolist()
		self.debug["rho"] = rho.flatten().tolist()
		self.debug["mu_qp"] = self.mu_qp.flatten().tolist()
		self.debug["mu"] = self.mu_new.flatten().tolist()
		self.debug["u_new"] = self.u_new.flatten().tolist()
		self.debug["u_unsat"] = u_new_unsaturated.flatten().tolist()
		self.debug["trueDelta"] = trueDelta.flatten().tolist()
		self.debug["true_predict_error"] = self.true_predict_error
		self.debug["mDelta"] = mDelta.flatten().tolist()
		self.debug["sigDelta"] = sigDelta.flatten().tolist()

		self.mu_prev = copy.copy(self.mu_new)
		self.u_prev_prev = copy.copy(self.u_prev)
		self.u_prev = copy.copy(self.u_new)
		self.obs_prev = copy.copy(self.obs)
		self.z_prev = copy.copy(self.z)

		self.controls = np.zeros(self.udim)
		self.controls[0] = np.arctan(self.u_new[0] * self.vehicle_length)
		self.controls[1] = self.u_new[1]
		return self.controls
