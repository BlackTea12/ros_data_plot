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
    # saving total data
    total_val1 = 0
    total_val2 = 0

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
    plt.ylim([0, 55])
    plt.xlim([7, 47])
    plt.xlabel('travel duration[sec]')
    plt.ylabel('execution time [ms]')
    plt.grid(axis='y')

    # Plot avg value
    time1 = []
    time2 = []
    for i in range(len(exec_time_time_list)):
        if exec_time_time_list[i] < 19 and exec_time_time_list[i] > 7:
            total_val1 += exec_time_list[i]
            time1.append(exec_time_time_list[i])
        elif exec_time_time_list[i] > 35 and exec_time_time_list[i] < 47:
            total_val2 += exec_time_list[i]
            time2.append(exec_time_time_list[i])

    total_val1 /= len(time1)
    total_val2 /= len(time2)
    y_avg1 = [total_val1 for i in range(len(time1))]
    y_avg2 = [total_val2 for i in range(len(time2))]
    plt.plot(time1, y_avg1, '--', color='#7CFC00')
    plt.plot(time2, y_avg2, '--', color='#7CFC00')

    # tip to annotate
    index_an1 = round(len(y_avg1)/2)
    index_an2 = round(len(y_avg2)/2)
    plt.annotate(f'avg: ({round(y_avg1[index_an1],2)})',
             xy=(time1[index_an1], y_avg1[index_an1]),
             xytext=(time1[index_an1] + 0.5, y_avg1[index_an1] - 3),
             arrowprops=dict(facecolor='black', shrink=0.05))
    plt.annotate(f'avg: ({round(y_avg2[index_an2],2)})',
             xy=(time2[index_an2], y_avg2[index_an2]),
             xytext=(time2[index_an2] + 0.5, y_avg2[index_an2] - 3),
             arrowprops=dict(facecolor='black', shrink=0.05))
    
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
