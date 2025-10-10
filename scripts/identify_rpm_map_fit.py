#!/usr/bin/python3
import argparse
import os
from itertools import product

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from sklearn.cluster import KMeans
from sklearn.neighbors import LocalOutlierFactor
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Define the function to fit
def func(X, a, b, c, d):
    x, y = X
    return a*y**b*(c*x*x+(1-c)*x)+d

if __name__ == "__main__":
    # Hyperparameters
    N_clusters = 50
    outlier_ratio = 0.3
    size_lookup_table = 200
    data_dir = '/home/charmyoung/flygripper_ws/scripts/data'
    # load data
    print("Loading data from directory %s" % data_dir)
    list_subfolders_with_paths = [f.path for f in os.scandir(data_dir) if f.is_dir()]
    print(list_subfolders_with_paths)
    esc_data = []
    for idx, directory in enumerate(list_subfolders_with_paths):
        if idx == 0:
            cmd_data = np.loadtxt(os.path.join(directory, "cmd_data.csv"), delimiter=',')[:, np.newaxis]
            for i in range(4):
                esc_data.append(
                    np.loadtxt(os.path.join(directory, "esc_data{}.csv".format(i)), delimiter=',')[:, np.newaxis])
            voltage_data = np.loadtxt(os.path.join(directory, "voltage_data.csv"), delimiter=',')[:, np.newaxis]
        else:
            cmd_data = np.concatenate(
                [cmd_data, np.loadtxt(os.path.join(directory, "cmd_data.csv"), delimiter=',')[:, np.newaxis]])
            for i in range(4):
                esc_data[i] = np.concatenate(
                [esc_data[i], np.loadtxt(os.path.join(directory, "esc_data{}.csv".format(i)), delimiter=',')[:, np.newaxis]])
            voltage_data = np.concatenate(
                [voltage_data, np.loadtxt(os.path.join(directory, "voltage_data.csv"), delimiter=',')[:, np.newaxis]])

    for i in range(4):
        X = np.concatenate([cmd_data, voltage_data, esc_data[i]**2], axis=1)

        print("Performing outlier removal...")
        # 1. Remove outlier
        clf = LocalOutlierFactor(n_neighbors=20, contamination='auto')
        y_pred = clf.fit_predict(X)
        X_scores = clf.negative_outlier_factor_
        # sort the scores and select the top k-percent quantile
        X_scores_sorted = np.sort(X_scores) # from small to large
        cutoff_score = X_scores_sorted[int(outlier_ratio * X_scores_sorted.shape[0])] # from small to large
        X_select = X_scores > cutoff_score # 0.3
        X_filtered = X[X_select] # select from 0.3 to 1.0
        print("Clustering data...")
        # apply kmeans for clustering and get clustered centers
        kmeans = KMeans(n_clusters=N_clusters, random_state=0).fit(X_filtered)
        X_clustered = kmeans.cluster_centers_

        # Flatten the 2D arrays
        # cmd_data_flat = X_clustered[:, 0].flatten()
        # voltage_data_flat = X_clustered[:, 1].flatten()
        # esc_data_flat = X_clustered[:, 2].flatten()
        cmd_data_flat = X[:, 0].flatten()
        voltage_data_flat = X[:, 1].flatten()
        esc_data_flat = X[:, 2].flatten()
        params_init = 0.1280, 1.3145, 0.2034, 0.5
        # Fit the function to the data using curve_fit
        params, covariance = curve_fit(func, (cmd_data_flat, voltage_data_flat), esc_data_flat)#, params_init)

        # Extract the fitted parameters
        a_fit, b_fit, c_fit, d_fit = params

        # Print the fitted parameters
        print('Fitted parameters:')
        print('a =', a_fit)
        print('b =', b_fit)
        print('c =', c_fit)
        print('d =', d_fit)

        # Evaluation
        for j in range(4):
            # rpm**2 = a*v**b*(c*u*u+(1-c)*u)+d
            z = esc_data[j]**2
            z_pred = func((cmd_data, voltage_data), a_fit, b_fit, c_fit, d_fit)
            # z_pred = func((cmd_data, voltage_data), *params_init)
            mse = np.mean((z_pred-z)**2)
            rmse = np.sqrt(mse)

            ss_total = np.sum((z - np.mean(z))**2)
            ss_residual = np.sum((z - z_pred)**2)
            rsquared = 1 - (ss_residual / ss_total)
            sse = ss_residual
            print("{} {} RMSE: {}, R²: {}, SSE: {}".format(i, j, rmse, rsquared, sse))

