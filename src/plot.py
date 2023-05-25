#!/usr/bin/env python3

import rospy
import rosbag
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

# get rosparam first
bag_path = rospy.get_param("rosbag_path_dir")
bag_name = rospy.get_param("rosbag_name")
img_path = rospy.get_param("img_path_dir")
def callback(data, t):
    if data._type == 'std_msgs/Float64':
        # Append the received Float64 data to a list
        pos_err_list.append(data.data*100)
        pos_err_time_list.append(t)
    elif data._type == 'nav_msgs/Odometry':
        # Extract twist.x field from Odometry message
        vel_list.append(data.twist.twist.linear.x)
        vel_time_list.append(t)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('plot_logger', anonymous=True)

    # Create a list to store the data
    pos_err_list = []
    pos_err_time_list = []
    vel_list = []
    vel_time_list = []

    # Specify the path to your rosbag file
    rosbag_name = bag_path + '/' + bag_name

    # Open the rosbag file
    bag = rosbag.Bag(rosbag_name)

    # Specify the topic you want to subscribe to
    topic_odom = '/odom'
    topic_pos_err = '/NaviFra/error_dist'

    # Subscribe to the topic and register the PosErrCallback function
    sub_odom = rospy.Subscriber(topic_odom, Odometry, callback)
    sub_pos_err = rospy.Subscriber(topic_pos_err, Float64, callback)

    # Process the messages in the rosbag
    for topic, msg, t in bag.read_messages(topics=[topic_odom, topic_pos_err]):
        callback(msg, t.to_sec())

    # Close the rosbag file
    bag.close()
    # filter time
    start_time_pos_err = pos_err_time_list[0]
    start_time_vel = vel_time_list[0]
    for i in range(len(pos_err_time_list)):
        pos_err_time_list[i] -= start_time_pos_err
    for i in range(len(vel_time_list)):
        vel_time_list[i] -= start_time_vel

    # Plot the data
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2, 1)
    # Plot the Float64 data
    ax1.plot(pos_err_time_list, pos_err_list, color='#9467bd')
    ax1.set_xlabel('time[sec]')
    ax1.set_ylabel('Robot Position Error [cm]')
    ax1.grid(axis='y')

    # Plot the twist.x data
    ax2.plot(vel_time_list, vel_list, color='#ff6969')
    ax2.set_xlabel('time[sec]')
    ax2.set_ylabel('Robot Linear Velocity [m/s]')
    ax2.grid(axis='y')

    # Adjust the layout of the subplots
    plt.legend(loc='best')
    fig.tight_layout()
    # Save the plot as an image file
    img_name = rosbag_name.split('.')
    img_name = img_name[0].split('/')
    print(img_name[-1])
    image_path = img_path+'/'+img_name[-1]+'.png'
    
    plt.savefig(image_path)

    # Show the plot
    plt.show()

    rospy.signal_shutdown("Plot completed")
