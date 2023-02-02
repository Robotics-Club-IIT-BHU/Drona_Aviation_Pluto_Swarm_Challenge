import os
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

dir = os.getcwd()
class Regression:
	def __init__(self):
		xy_lin1, xy_poly2, xy_lin2 = self.train_model(xy_X, xy_Y)
		z_lin1, z_poly2, z_lin2 = self.train_model(z_X, z_Y)
		self.xy_lin1, self.z_lin1 = xy_lin1, z_lin1
		self.xy_poly2, self.z_poly2 = xy_poly2, z_poly2
		self.xy_lin2, self.z_lin2 = xy_lin2, z_lin2
	def train_model(self, X, Y):
		lin1 = LinearRegression()
		lin1.fit(X, Y)
		poly2 = PolynomialFeatures(degree=4)
		poly2_out = poly2.fit_transform(X)
		poly2.fit(poly2_out, Y)
		lin2 = LinearRegression()
		lin2.fit(poly2_out, Y)
		return lin1, poly2, lin2
	def linear_regression(self, data):
		xy = self.xy_lin1.predict(data)[0]
		z = self.z_lin1.predict(data)[0]
		pos = np.concatenate((xy,z), axis=-1).flatten()
		return pos
		return np.concatenate((xy, z), axis=-1)
	def polynomial_regression(self, data):
		xy = self.xy_lin2.predict(self.xy_poly2.fit_transform(data))
		z = self.z_lin2.predict(self.z_poly2.fit_transform(data))
		pos = np.concatenate((xy,z), axis=-1).flatten()
		return pos

xy_dataset = pd.read_csv(dir+"/localization/dataset/xy_dataset.csv")
xy_X = xy_dataset.iloc[:, :-2].values
xy_Y = xy_dataset.iloc[:, -2:].values
z_dataset = pd.read_csv(dir+"/localization/dataset/z_dataset.csv")
z_X = z_dataset.iloc[:, :-1].values
z_Y = z_dataset.iloc[:, -1:].values
