import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

class Regression:
	def __init__(self):
		lin1, poly, lin2 = self.train_model(X, Y)
		self.lin1 = lin1
		self.poly = poly
		self.lin2 = lin2
	def simple_regression(self, tvec):
		m = np.array([0.8928, 0.92259, 0]
		_tvec = tvec.flatten()
		_tvec = np.multiply(_tvec, m)
		return _tvec
