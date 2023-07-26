#!/usr/bin/env python3

import rospy
import rosbag
import matplotlib.pyplot as plt
from std_msgs.msg import Float64

# get rosparam first
bag_path = rospy.get_param("rosbag_path_dir")
bag_name = rospy.get_param("rosbag_name")
img_path = rospy.get_param("img_path_dir")
def callback(data, t):
    if data._type == 'std_msgs/Float64':
        # Append the received Float64 data to a list
        exec_time_list.append(data.data)
        exec_time_time_list.append(t)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('plot_logger', anonymous=True)

    # Create a list to store the data
    exec_time_list = []
    exec_time_time_list = []

    # Specify the path to your rosbag file
    rosbag_name = bag_path + '/' + bag_name

    # Open the rosbag file
    bag = rosbag.Bag(rosbag_name)

    # Specify the topic you want to subscribe to
    topic_time_exec = '/NaviFra/exec_time_ms'

    # Subscribe to the topic and register the PosErrCallback function
    sub_exec_time = rospy.Subscriber(topic_time_exec, Float64, callback)

    # Process the messages in the rosbag
    for topic, msg, t in bag.read_messages(topics=[topic_time_exec]):
        callback(msg, t.to_sec())

    # Close the rosbag file
    bag.close()
    # filter time
    start_time_pos_err = exec_time_time_list[0]
    for i in range(len(exec_time_time_list)):
        exec_time_time_list[i] -= start_time_pos_err

    # Plot the data
    # Create subplots
    # Plot the Float64 data
    plt.plot(exec_time_time_list, exec_time_list, color='#9467bd')
    # plt.ylim([0, 50])
    plt.xlabel('travel duration[sec]')
    plt.ylabel('execution time [ms]')
    plt.grid(axis='y')

    # Adjust the layout of the subplots
    plt.legend(loc='best')
    plt.tight_layout()
    # Save the plot as an image file
    img_name = rosbag_name.split('.')
    img_name = img_name[0].split('/')
    print(img_name[-1])
    image_path = img_path+'/'+img_name[-1]+'.png'
    
    plt.savefig(image_path)

    # Show the plot
    plt.show()

    rospy.signal_shutdown("Plot completed")
