# ros_data_plot
_Custom python plot package for ROS using matplotlib_

## How to record your topic
In your command prompt,
`
rosbag record {your_topic_name}
`

## How to plot your data (plot.py)
In this script, two messages are used for plot.
Both type are **'std_msgs/Float64'** and **'nav_msgs/Odometry'**.

First go to '/launch' Folder.
There are three parameters you can modify.
1. rosbag_path_dir -> Get your rosbag file in folder '/rosbag'
2. img_path_dir -> Save your plot image in folder '/img'
3. rosbag_name -> Fetch your rosbag with name (Please modify to your actual rosbag name)
