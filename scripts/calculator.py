import warnings
# Filter
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning)

import math
import numpy as np
# Static constant
kRadPerSecToRpm = 30.0 / math.pi
kRadToDeg = 180.0 / math.pi
kGravity = 9.7936
kGeneralTorqueForceRatio = 0.016

# ------------------------------------User input--------------------------------------------
# mass = 0.712
# inertia = np.array([0.007, 0.007, 0.012])
# thrust_coeff_rpm=9.3745e-08
# x_pos = 0.12
# y_pos = 0.12

mass = 0.485
inertia = np.array([0.006, 0.006, 0.03])
thrust_coeff_rpm=4.76e-9
x_pos = 0.145/2
y_pos = 0.175/2

k=[0.0094, -8.746306995940205e-06, 1.791322293868279e-09]
guess_k = False #True
plot_k = True #True

use_min_max_coeff = True
# CONDITION 1:
# use_min_max_coeff: True
# given min_coeff and max_coeff, calculate min_thrust and max_thrust
min_coeff = 0.2
max_coeff = 2.5 # Thrust-Weight-Ratio
# CONDITION 2:
# use_min_max_coeff: False
# given rotor_rpm_min and rotor_rpm_max, calculate rotor_thrust_min and rotor_thrust_max
rotor_rpm_max = 26000
rotor_rpm_min = 6000


# --------------------------------------------------------------------------------------------
print("mass: {:.3f}".format(mass))
weight = mass * kGravity
print("weight: {:.3f}".format(weight))

print("thrust_coeff(rpm): {:.3e}".format(thrust_coeff_rpm))
thrust_coeff_rps = thrust_coeff_rpm * (kRadPerSecToRpm * kRadPerSecToRpm)
print("thrust_coeff(rps): {:.3e}".format(thrust_coeff_rps))

torque_coeff_rpm = kGeneralTorqueForceRatio*thrust_coeff_rpm
print("torque_coeff(rpm): {:.3e}".format(torque_coeff_rpm))
torque_coeff_rps = torque_coeff_rpm * (kRadPerSecToRpm * kRadPerSecToRpm)
print("torque_coeff(rps): {:.3e}".format(torque_coeff_rps))

if use_min_max_coeff:
    min_thrust = min_coeff * weight
    max_thrust = max_coeff * weight
    print("min_thrust with {:.0%} weight: {:.5f}".format(min_coeff, min_thrust))
    print("max_thrust with {:.0%} weight: {:.5f}".format(max_coeff, max_thrust))
    rotor_thrust_min = min_thrust / 4
    rotor_thrust_max = max_thrust / 4
    print("rotor_thrust_min: [{:.5f}, {:.5f}, {:.5f}, {:.5f}]".format(
        rotor_thrust_min, rotor_thrust_min, rotor_thrust_min, rotor_thrust_min))
    print("rotor_thrust_max: [{:.5f}, {:.5f}, {:.5f}, {:.5f}]".format(
        rotor_thrust_max, rotor_thrust_max, rotor_thrust_max, rotor_thrust_max))
    rotor_rpm_max = math.sqrt(rotor_thrust_max/thrust_coeff_rpm)
    rotor_rps_max = rotor_rpm_max / kRadPerSecToRpm
    rotor_rpm_min = math.sqrt(rotor_thrust_min/thrust_coeff_rpm)
    rotor_rps_min = rotor_rpm_min / kRadPerSecToRpm

    print("rotor_max_speed(rpm): {:.5f}".format(rotor_rpm_max))
    print("rotor_min_speed(rpm): {:.5f}".format(rotor_rpm_min))

    print("rotor_max_speed(rps): {:.5f}".format(rotor_rps_max))
    print("rotor_min_speed(rps): {:.5f}".format(rotor_rps_min))
else:
    print("rotor_max_speed(rpm): {:.3f}".format(rotor_rpm_max))
    print("rotor_min_speed(rpm): {:.3f}".format(rotor_rpm_min))

    rotor_rps_min = rotor_rpm_min / kRadPerSecToRpm
    rotor_rps_max = rotor_rpm_max / kRadPerSecToRpm
    print("rotor_max_speed(rps): {:.3f}".format(rotor_rps_max))
    print("rotor_min_speed(rps): {:.3f}".format(rotor_rps_min))

    rotor_thrust_min = thrust_coeff_rpm * rotor_rpm_min * rotor_rpm_min
    rotor_thrust_max = thrust_coeff_rpm * rotor_rpm_max * rotor_rpm_max
    print("rotor_thrust_min: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(
        rotor_thrust_min, rotor_thrust_min, rotor_thrust_min, rotor_thrust_min))
    print("rotor_thrust_max: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(
        rotor_thrust_max, rotor_thrust_max, rotor_thrust_max, rotor_thrust_max))
    min_thrust = rotor_thrust_min * 4
    max_thrust = rotor_thrust_max * 4
    min_coeff = min_thrust / weight
    max_coeff = max_thrust / weight
    print("min_thrust with {:.0%} weight: {:.3f}".format(min_coeff, min_thrust))
    print("max_thrust with {:.0%} weight: {:.3f}".format(max_coeff, max_thrust))

G = np.array([[1.0, 1.0, 1.0, 1.0],
              [-y_pos, y_pos, y_pos, -y_pos],
              [x_pos, -x_pos, x_pos, -x_pos],
              [kGeneralTorqueForceRatio,kGeneralTorqueForceRatio,-kGeneralTorqueForceRatio,-kGeneralTorqueForceRatio]])
print("G_inv: ", np.linalg.inv(G))
max_torque_x_thrust = np.array([rotor_thrust_min, rotor_thrust_max, rotor_thrust_max, rotor_thrust_min]).reshape([-1,1])
max_torque_x = (G @ max_torque_x_thrust).flatten()

max_torque_y_thrust = np.array([rotor_thrust_max, rotor_thrust_min, rotor_thrust_max, rotor_thrust_min]).reshape([-1,1])
max_torque_y = (G @ max_torque_y_thrust).flatten()

max_torque_z_thrust = np.array([rotor_thrust_max, rotor_thrust_max, rotor_thrust_min, rotor_thrust_min]).reshape([-1,1])
max_torque_z = (G @ max_torque_z_thrust).flatten()

# print("max_torque_x: ", max_torque_x)
# print("max_torque_y: ", max_torque_y)
# print("max_torque_z: ", max_torque_z)
max_torque = np.array([max_torque_x[1], max_torque_y[2], max_torque_z[3]])
print("max_torque: [{:.3f}, {:.3f}, {:.3f}]".format(max_torque[0], max_torque[1], max_torque[2]))
max_angular_acc = max_torque / inertia * kRadToDeg
print("max_angular_acc: [{:.3f}, {:.3f}, {:.3f}]".format(max_angular_acc[0], max_angular_acc[1], max_angular_acc[2]))
#----------------------------------- dshot rpm mapping ----------------------------------------
import numpy as np
# [k0, k1, k2], y = k0 + k1 * x + k2 * x * x
# where x is rpm, y is dshot command

if guess_k:
    from scipy.optimize import fsolve

    # 生成一个映射
    def equation(k):
        global rotor_rpm_max, rotor_rpm_min
        k0, k1, k2 = k
        eq0 = -k1/(2*rotor_rpm_min)-k2
        eq1 = k0 + k1 * rotor_rpm_max + k2 * rotor_rpm_max**2 - 1.0
        eq2 = k0 + k1 * rotor_rpm_min + k2 * rotor_rpm_min**2
        return [eq0, eq1, eq2]

    # 已知的最大值和最小值对应的 dshot 和 rpm
    # 初始猜测值
    k_guess = [0, 0, 0]

    # 解方程
    k = fsolve(equation, k_guess)

# 输出结果
print("k0, k1, k2: [{:.3e}, {:.3e}, {:.3e}]".format(k[0], k[1], k[2]))


if plot_k:
    import matplotlib.pyplot as plt
    def rpm_from_dshot(dshot, k):
        k0, k1, k2 = k
        discriminant = k1**2 - 4*k2*(k0 - dshot)
        #取正的部分
        # discriminant = discriminant.clip(min=0)
        return (-k1 + np.sqrt(discriminant)) / (2 * k2)

    def dshot_from_rpm(rpm, k):
        k0, k1, k2 = k
        return k0 + k1 * rpm + k2 * rpm * rpm
    # k = [0.0094, -8.746306995940205e-06, 1.791322293868279e-09]
    # dshot_values = np.linspace(0.01, 0.99, 1000)  # DShot范围：0到1，分成1000个点
    # rpm_values = rpm_from_dshot(dshot_values, k)

    rpm_values = np.linspace(rotor_rpm_min, rotor_rpm_max, 1000)
    dshot_values = dshot_from_rpm(rpm_values, k)

    # plot
    plt.plot(rpm_values, dshot_values)
    plt.title('RPM vs DShot Command \n(RPM min: {:.2f}, RPM max: {:.2f})'.format(np.min(rpm_values), np.max(rpm_values)))
    # plt.xlabel('DShot Command')
    # plt.ylabel('RPM')
    plt.xlabel('RPM')
    plt.ylabel('DShot Command')
    plt.grid(True)
    plt.show()