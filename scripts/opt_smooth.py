import numpy as np
import matplotlib.pyplot as plt
import math
def smoothedL1(x):
    mu = 0.01
    if x < 0.0:
        grad = 0.0
        return 0.0, grad
    elif x > mu:
        grad = 1.0
        return x - 0.5 * mu, grad
    else:
        xdmu = x / mu
        sqrxdmu = xdmu * xdmu
        mumxd2 = mu - 0.5 * x
        grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu)
        return mumxd2 * sqrxdmu * xdmu, grad


def smoothed01(t):
    mu = 0.01
    mu4 = mu * mu * mu * mu
    mu4_1 = 1.0 / mu4
    if t < -mu:
        grad = 0
        return 0, grad
    elif t < 0:
        y = t + mu
        y2 = y * y
        grad = y2 * (mu - 2 * t) * mu4_1
        return 0.5 * y2 * y * (mu - t) * mu4_1, grad
    elif t < mu:
        y = t - mu
        y2 = y * y
        grad = y2 * (mu + 2 * t) * mu4_1
        return 0.5 * y2 * y * (mu + t) * mu4_1 + 1, grad
    else:
        grad = 0
        return 1, grad


def expC2(t):
    if t > 0.0:
        return ((0.5 * t + 1.0) * t + 1.0)
    else:
        return 1.0 / ((0.5 * t - 1.0) * t + 1.0)


def logC2(t):
    if t > 1.0:
        return math.sqrt(2.0 * t - 1.0) - 1.0
    else:
        return 1.0 - math.sqrt(2.0 / t - 1.0)

# 绘制图像
plt.figure(figsize=(10, 10))

# 生成一组 t 值
t_values = np.linspace(-5, 5, 1000)

# 计算 expC2 对应的函数值
expC2_values = [expC2(t) for t in t_values]

plt.subplot(2, 2, 1)
plt.plot(t_values, expC2_values)
plt.xlabel('t')
plt.ylabel('Gradient')
plt.title('Plot of expC2')
plt.legend()
plt.grid(True)

# 生成一组 t 值
t_values = np.linspace(0.01, 5, 1000)

# 计算 expC2 对应的函数值
logC2_values = [logC2(t) for t in t_values]

plt.subplot(2, 2, 2)
plt.plot(t_values, logC2_values)
plt.xlabel('t')
plt.ylabel('Gradient')
plt.title('Plot of logC2')
plt.legend()
plt.grid(True)

# 生成一组 t 值
t_values = np.linspace(-0.02, 0.02, 1000)
# 计算 smoothed01 对应的函数值和梯度值
smoothed01_ret = [smoothed01(t) for t in t_values]
smoothed01_values = [val[0] for val in smoothed01_ret]
smoothed01_grad = [val[1] for val in smoothed01_ret]

plt.subplot(2, 2, 3)
plt.plot(t_values, smoothed01_values)
plt.xlabel('t')
plt.ylabel('Function value')
plt.title('Plot of smoothed01')
plt.legend()
plt.grid(True)


# 生成一组 t 值
t_values = np.linspace(-0.02, 0.02, 1000)
# 计算 smoothed01 对应的函数值和梯度值
smoothedL1_ret = [smoothedL1(t) for t in t_values]
smoothedL1_values = [val[0] for val in smoothedL1_ret]
smoothedL1_grad = [val[1] for val in smoothedL1_ret]

plt.subplot(2, 2, 4)
plt.plot(t_values, smoothedL1_values)
plt.xlabel('t')
plt.ylabel('Function value')
plt.title('Plot of smoothedL1')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()