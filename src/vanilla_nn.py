import tensorflow as tf
import keras
from keras.models import Model
from keras.layers import Input, Layer, Dense, Dropout, Activation
from keras import backend as K

class NegativeLogLikelihood(Layer):
	def call(self, inputs):
		y_pred_mean = inputs[0]
		y_pred_var = inputs[1]
		y_true = inputs[2]
		loss = K.sum(K.log(y_pred_var+K.epsilon())+K.square(y_pred_mean-y_true) / y_pred_var,axis=-1)
		return loss

def identity_loss(y_true, loss):
	return loss

class VanillaNN:
	def __init__(self,xdim,ydim):
		input_x = Input(shape=(xdim,), dtype='float', name='input_x')
		x = Dense(256, activation='tanh')(input_x)
		x = Dropout(0.5)(x)
		x = Dense(256, activation='tanh')(x)
		x = Dropout(0.5)(x)
		x = Dense(256, activation='tanh')(x)
		x = Dropout(0.5)(x)
		x = Dense(256, activation='tanh')(x)
		output_mean = Dense(ydim, activation='linear', name='output_mean')(x)
		output_var = Dense(ydim, activation='softplus', name='output_var')(x)
		input_y = Input(shape=(ydim,), dtype='float', name='input_y')
		output_loss = NegativeLogLikelihood()([output_mean,output_var,input_y])

		self.m = Model(inputs=[input_x,input_y], outputs=[output_mean,output_var,output_loss])
		rmsprop = keras.optimizers.RMSprop(lr=0.003, rho=0.9, epsilon=None, decay=0.0)
		self.m.compile(loss=identity_loss, loss_weights=[0., 0., 1.], optimizer=rmsprop)

		self.graph = tf.get_default_graph()

	def train(self, Z_train, Y_train,_epoch=50,_batch_size=64,_verbose=0):
		with self.graph.as_default():
			self.m.fit([Z_train,Y_train], [Y_train,Y_train,Y_train],epochs=_epoch,batch_size=_batch_size,verbose=_verbose)

	def predict(self, ZN, Y, _batch_size=1):
		with self.graph.as_default():
			y, var, loss = self.m.predict([ZN,Y], batch_size=_batch_size)
		# print("Predicted y ", y)
		# print("predicted var ", var)
		# print("predicted loss ", loss)
		return y[0], var[0], loss[0]
