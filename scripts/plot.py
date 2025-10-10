import rosbag
import matplotlib.pyplot as plt
import math
import numpy as np
from flygripper_msgs.msg import GeometricControllerData  # Adjust the import based on your message type

# def read_rosbag_and_plot():
#     # Lists to store data for each group
#     data = []
#     # Open the bag file
#     bag_file_path = '/home/flygripperyoung/workspace/flygripper_robotics/NONE_PL_v3.bag'
#     with rosbag.Bag(bag_file_path, 'r') as bag:
#         xy_rmses = []
#         z_rmses = []
#         for topic, msg, t in bag.read_messages(topics=['/flygripper_simple/geometric_controller/data']):
#             xy_rmse = math.sqrt(msg.position_error.x**2 + msg.position_error.y**2)
#             z_rmse = math.sqrt(msg.position_error.z**2)
#             xy_rmses.append(xy_rmse)
#             z_rmses.append(z_rmse)
#         xy_rmses_array = np.array(xy_rmses)
#         z_rmses_array = np.array(z_rmses)
#         print("xy: ", xy_rmses_array.mean(), xy_rmses_array.var())
#         print("z: ", z_rmses_array.mean(), z_rmses_array.var())
#
#     # Create a histogram to visualize the data distribution
#     # plt.hist(data, bins=2, alpha=0.5, color=['blue','red'], label='Data Distribution')
#
#     # Plot the mean as a horizontal line
#     # plt.axvline(mean_value, color='red', linestyle='dashed', linewidth=2, label=f'Mean: {mean_value:.2f}')
#
#     # # Set plot labels and title
#     # plt.xlabel('Groups')
#     # plt.ylabel('Values')
#     # plt.title('Box Plot of ROS Message Data')
#     #
#     # # Show the plot
#     # plt.show()


def read_rosbag_and_plot():
    # Lists to store data for each group
    data = []
    # Open the bag file
    bag_file_path = '/home/flygripperyoung/workspace/flygripper_robotics/NONE_PL_v3.bag'
    with rosbag.Bag(bag_file_path, 'r') as bag:
        xy_rmses = []
        z_rmses = []
        for topic, msg, t in bag.read_messages(topics=['/flygripper_simple/geometric_controller/data']):
            xy_rmse = math.sqrt(msg.position_error.x**2 + msg.position_error.y**2)
            z_rmse = math.sqrt(msg.position_error.z**2)
            xy_rmses.append(xy_rmse)
            z_rmses.append(z_rmse)
        xy_rmses_array = np.array(xy_rmses)
        z_rmses_array = np.array(z_rmses)
        print("xy: ", xy_rmses_array.mean(), xy_rmses_array.var())
        print("z: ", z_rmses_array.mean(), z_rmses_array.var())

    # Create a histogram to visualize the data distribution
    # plt.hist(data, bins=2, alpha=0.5, color=['blue','red'], label='Data Distribution')

    # Plot the mean as a horizontal line
    # plt.axvline(mean_value, color='red', linestyle='dashed', linewidth=2, label=f'Mean: {mean_value:.2f}')

    # # Set plot labels and title
    # plt.xlabel('Groups')
    # plt.ylabel('Values')
    # plt.title('Box Plot of ROS Message Data')
    #
    # # Show the plot
    # plt.show()

if __name__ == '__main__':
    try:
        read_rosbag_and_plot()
    except Exception as e:
        print(f"Error: {e}")