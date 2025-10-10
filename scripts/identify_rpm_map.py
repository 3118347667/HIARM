#!/usr/bin/python3
import argparse
import os
from itertools import product

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from sklearn.cluster import KMeans
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel, WhiteKernel
from sklearn.neighbors import LocalOutlierFactor

if __name__ == "__main__":
    # Hyperparameters
    N_clusters = 1000
    outlier_ratio = 0.2
    size_lookup_table = 100
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

    # filter
    esc_data_flt = []
    esc_data_flt1 = []
    for i in range(4):
        # using mean for all values
        esc_bucket_sum = 0
        esc_bucket_start_index = 0
        esc_bucket_end_index = 0
        esc_data_flt.append(esc_data[i].copy())
        for j in range(len(esc_data[i])):
            if j >= 1:
                if abs(cmd_data[j] - cmd_data[j - 1]) > 1e-2:
                    esc_bucket_end_index = j
                    esc_bucket_flt = esc_bucket_sum / (esc_bucket_end_index - esc_bucket_start_index)
                    for k in range(esc_bucket_start_index, esc_bucket_end_index):
                            esc_data_flt[i][k] = esc_bucket_flt
                    esc_bucket_start_index = j
                    esc_bucket_sum = 0
            esc_bucket_sum = esc_bucket_sum + esc_data[i][j]

        # using lpf
        esc_bucket_flt = esc_data[i][0]
        esc_data_flt1.append(esc_data[i].copy())
        for j in range(len(esc_data[i])):
            if j >= 1:
                if abs(cmd_data[j] - cmd_data[j - 1]) > 1e-2:
                    esc_bucket_flt = esc_data_flt[i][j]
            esc_data_flt1[i][j] = 0.9 * esc_bucket_flt + 0.1 * esc_data_flt1[i][j]
            esc_bucket_flt = esc_data_flt1[i][j]

        fig, ax = plt.subplots(1, 1)
        ax.scatter(cmd_data, esc_data[i],  color='g', s=1., label='raw esc data points')
        ax.scatter(cmd_data, esc_data_flt1[i], color='b', s=3., label='lpf filter esc data points')
        # ax.scatter(cmd_data, esc_data_flt[i], color='r', s=5., label='mean filter esc data points')
        ax.set_xlabel('Command {} [-]'.format(i))
        ax.set_ylabel('Rotor Speed {} [RPM]'.format(i))
        ax.legend()
        plt.show()

    for i in range(4):
        # apply lpf for esc data
        X = np.concatenate([cmd_data, voltage_data, esc_data_flt1[i]], axis=1)
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
        kmeans = KMeans(n_clusters=N_clusters, n_init=10, random_state=0).fit(X_filtered)
        X_clustered = kmeans.cluster_centers_

        print("Fitting data...")
        X_gp = X_clustered[:, 1:] # voltage, esc
        y_gp = X_clustered[:, 0] # cmd
        # X_gp = X_filtered[:, 1:] # voltage, esc
        # y_gp = X_filtered[:, 0]  # cmd
        # apply gaussian process regressor
        kernel = ConstantKernel(1.0, (1e-3, 1e3)) * RBF([1, 1], (1.0e-5, 1.0e5)) \
                 + WhiteKernel(noise_level=1e-2,
                               noise_level_bounds=(1e-3, 1e+1))
        gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=15)
        gp.fit(X_gp, y_gp)
        # ---------------------------------------- Evaluation ------------------------------------------------------
        for j in range(4):
            X = np.concatenate([cmd_data, voltage_data, esc_data[j]], axis=1)
            y_pred, MSE = gp.predict(X[:, 1:], return_std=True)
            mse = np.mean((y_pred-X[:, 0])**2)
            rmse = np.sqrt(mse)

            ss_total = np.sum((X[:, 0] - np.mean(X[:, 0]))**2)
            ss_residual = np.sum((X[:, 0] - y_pred)**2)
            rsquared = 1 - (ss_residual / ss_total)
            sse = ss_residual
            print("{} {} RMSE: {}, R²: {}, SSE: {}".format(i, j, rmse, rsquared, sse))

        # -------------------------------------- Plotting -------------------------------------------------

        # Input space
        x1 = np.linspace(X_clustered[:, 1].min(), X_clustered[:, 1].max(), size_lookup_table)  # voltage
        x2 = np.linspace(X_clustered[:, 2].min(), X_clustered[:, 2].max(), size_lookup_table)  # esc
        x1x2 = np.array(list(product(x1, x2)))
        y_pred, MSE = gp.predict(x1x2, return_std=True)
        X0p = x1x2[:, 0].reshape(size_lookup_table, size_lookup_table)
        X1p = x1x2[:, 1].reshape(size_lookup_table, size_lookup_table)
        Zp = np.reshape(y_pred, (size_lookup_table, size_lookup_table))

        print("Plotting results...")
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(X0p, X1p, Zp, rstride=1, cstride=1, cmap='gist_rainbow', linewidth=0, antialiased=True)
        ax.scatter(voltage_data, esc_data[i], cmd_data, color='k', s=1., label='raw Data Points')
        ax.scatter(X_filtered[:, 1], X_filtered[:, 2], X_filtered[:, 0], color='b', s=10., label='Filtered Data Points')
        ax.scatter(X_clustered[:, 1], X_clustered[:, 2], X_clustered[:, 0], color='r', s=50., label='Clustered Data Points')
        ax.set_xlabel('Voltage [V]')
        ax.set_ylabel('Rotor Speed {} [RPM]'.format(i))
        ax.set_zlabel('Command {} [-]'.format(i))
        plt.show()

        fig, ax = plt.subplots(1, 1)
        step = 0.01
        m = np.amax(Zp)
        levels = np.arange(0.0, m, step) + step

        cp = ax.contourf(X0p, X1p, Zp, levels, cmap='jet', alpha=0.8)
        fig.colorbar(cp)
        ax.scatter(X_filtered[:, 1], X_filtered[:, 2],
                   color='k', s=1., label='Filtered data points')
        ax.set_title('Command {} Contour Plot'.format(i))
        ax.set_xlabel('Voltage [V]')
        ax.set_ylabel('Rotor Speed {} [RPM]'.format(i))
        plt.show()

        # ------------------------------------- Generate lookup table --------------------------------------------
        lookup_table_fname = os.path.join(data_dir, 'rpm{}_map.csv'.format(i))
        print(
            "Saving lookup table of size [%dx%d] to [%s] ..." % (size_lookup_table, size_lookup_table, lookup_table_fname))
        print("Rows denote voltage in the range [%.2f, %.2f] V in increasing order." % (np.min(X0p), np.max(X0p)))
        print("Columns denote rotor speed in the range [%.2f, %.2f] RPM in increasing order." % (np.min(X1p), np.max(X1p)))

        # we add some metainformation at the beginning row of the rpm map
        meta_info = np.zeros((1, size_lookup_table))
        meta_info[0, 0] = size_lookup_table
        meta_info[0, 1] = np.min(X1p) # esc min
        meta_info[0, 2] = np.max(X1p) # esc max
        meta_info[0, 3] = np.min(X0p) # voltage min
        meta_info[0, 4] = np.max(X0p) # voltage max
        Zp = np.concatenate([meta_info, Zp], axis=0)
        np.savetxt(lookup_table_fname, Zp, delimiter=',', fmt='%f')

    print("Done!")


