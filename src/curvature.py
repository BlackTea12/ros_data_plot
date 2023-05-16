#!/usr/bin/env python3

import rospy
import rosbag
import math
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# get rosparam first
bag_path = rospy.get_param("rosbag_path_dir")
bag_name = rospy.get_param("rosbag_name")
img_path = rospy.get_param("img_path_dir")
data = Twist()
data._type
def callback(data, t):
    if data._type == 'geometry_msgs/Twist':
        # Append the Twist->linear->x & Twist->angular->z for linear and angular velocity
        cmd_v.append(data.linear.x)
        cmd_w.append(data.angular.z*180/3.14)
        cmd_time.append(t)
    elif data._type == 'nav_msgs/Odometry':
        # Append Odometry->pose->pose->position->x & Odometry->pose->pose->position->y for x, y position
        pos_x.append(data.pose.pose.position.x)
        pos_y.append(data.pose.pose.position.y)
        pos_time.append(t)

def set_curvature(cur_x, cur_y, prev_x, prev_y):
    answer = 0.0
    den = abs(dx[0]*dx[0] + dy[0]*dy[0])
    if den == 0.0:
        answer = 0.0
    else:
        nom = abs(dx[0]*(dy[0]-dy[1]) - dy[0]*(dx[0]-dx[1]))
        denom = math.pow(math.sqrt(den),3)
        answer = nom/denom
    
    # save past data
    dx[1] = dx[0]
    dy[1] = dy[0]
    dx[0] = cur_x - prev_x
    dy[0] = cur_y - prev_y
    return answer

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('plot_logger', anonymous=True)

    # Create a list to store the data
    pos_x = []
    pos_y = []
    pos_time = []
    cmd_v = []
    cmd_w = []
    cmd_time = []
    
    # past data saving
    dx = [0.0, 0.0]
    dy = [0.0, 0.0]

    # Specify the path to your rosbag file
    rosbag_name = bag_path + '/' + bag_name

    # Open the rosbag file
    bag = rosbag.Bag(rosbag_name)

    # Specify the topic you want to subscribe to
    topic_odom = '/odom'
    topic_vel_cmd = '/cmd_vel'

    # Subscribe to the topic and register the PosErrCallback function
    sub_odom = rospy.Subscriber(topic_odom, Odometry, callback)
    sub_pos_err = rospy.Subscriber(topic_vel_cmd, Twist, callback)

    # Process the messages in the rosbag
    for topic, msg, t in bag.read_messages(topics=[topic_odom, topic_vel_cmd]):
        callback(msg, t.to_sec())

    # Close the rosbag file
    bag.close()

    # filter time
    start_time_pos = pos_time[0]
    start_time_cmd = cmd_time[0]
    for i in range(len(pos_time)):
        pos_time[i] -= start_time_pos
    for i in range(len(cmd_time)):
        cmd_time[i] -= start_time_cmd

    # get curvature value based on recorded position of robot
    curv_result = []
    for i in range(1, len(pos_x)):
        curv_result.append(set_curvature(pos_x[i], pos_y[i], pos_x[i-1], pos_y[i-1]))

    # Plot the data
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2, 1)
    fig.suptitle('Angular Velocity Set to Max Value: 30[deg/s]', fontsize=16, fontweight='bold', va='top')
    # fig.legend(loc='best')
    # fig.subplots_adjust(top=0.5)
    # fig.tight_layout()

    # Plot the Float64 data
    ax1.plot(pos_time[1:], curv_result, color='#d1193e')
    ax1.set_xlabel('time[sec]', fontweight='bold')
    ax1.set_ylabel('Curvature Radius', fontweight='bold')
    ax1.grid(axis='y')
    

    # Plot the twist.x data
    ax2.plot(cmd_time, cmd_v, color='#3a0751')
    # ax2.plot(cmd_time, cmd_w, color='#3a0751')
    ax2.set_xlabel('time[sec]', fontweight='bold')
    ax2.set_ylabel('Robot Linear Velocity [m/s]', fontweight='bold')
    ax2.grid(axis='y')

    # Adjust the layout of the subplots
    fig.tight_layout()
    fig.legend(loc='best')

    # Save the plot as an image file
    img_name = rosbag_name.split('.')
    img_name = img_name[0].split('/')
    print(img_name[-1])
    image_path = img_path+'/'+img_name[-1]+'.png'
    
    plt.savefig(image_path)

    # Show the plot
    plt.show()

    rospy.signal_shutdown("Plot completed")
