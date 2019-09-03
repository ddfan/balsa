import numpy as np
import matplotlib.pyplot as plt
import GPy


class ScaledGP:
	def __init__(self,xdim=1,ydim=1):
		self.xdim=xdim
		self.ydim=ydim
		self.ystd = np.ones(ydim)
		self.ymean = np.zeros(ydim)
		self.xstd = np.ones(xdim)
		self.xmean = np.zeros(xdim)
		self.m = GPy.models.GPRegression(np.zeros((1,xdim)),np.zeros((1,ydim)))

	def optimize(self,x,y,update_scaling=True, num_inducing=50):
		assert(x.shape[1] == self.xdim and y.shape[1] == self.ydim)
		assert(x.shape[0] > 0 and y.shape[0] > 0)
		
		xmean = self.xmean
		xstd = self.xstd
		ymean = self.ymean
		ystd = self.ystd
		if update_scaling:
			xmean,xstd = self.update_xscale(x)
			ymean,ystd = self.update_yscale(y)

		x = self.scalex(x,xmean,xstd)
		y = self.scaley(y,ymean,ystd)
		updated_model = GPy.models.GPRegression(x,y)
		# self.m = GPy.models.SparseGPRegression(x,y,num_inducing=num_inducing)
		updated_model.optimize('bfgs')
		self.m = updated_model

		self.xmean = xmean
		self.xstd = xstd
		self.ymean = ymean
		self.ystd = ystd

	def predict(self,x):
		x = self.scalex(x,self.xmean,self.xstd)
		mean,var = self.m.predict_noiseless(x)
		mean = self.unscaley(mean,self.ymean,self.ystd)
		var = var * self.ystd
		if mean.size == 1:
			mean = mean[0,0]
			var = var[0,0]
		return mean,var

	def update_xscale(self,x):
		xmean = np.mean(x,axis=0)
		xstd = np.std(x,axis=0)

		return xmean,xstd

	def update_yscale(self,y):
		ymean = np.mean(y,axis=0)
		ystd = np.std(y,axis=0)

		return ymean, ystd

	def scalex(self,x,xmean,xstd):
		if (xstd == 0).any():
			return (x-xmean)
		else:
			return (x - xmean) / xstd

	def scaley(self,y,ymean,ystd):
		if (ystd == 0).any():
			return (y-ymean)
		else:
			return (y - ymean) / ystd
		
	def unscalex(self,x,xmean,xstd):
		if (xstd == 0).any():
			return x + xmean
		else:
			return x * xstd + xmean

	def unscaley(self,y,ymean,ystd):
		if (ystd == 0).any():
			return y + ymean
		else:
			return y * ystd + ymean