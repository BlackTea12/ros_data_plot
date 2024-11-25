# Data Plot
<div align="right">

  <a href="">![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-green)</a>
  <a href="">![ROS](https://img.shields.io/badge/ROS-noetic-blue)</a>

  <p>Custom python plot package for ROS using matplotlib</p>
</div>



## 📖Descriptions
### 1. How to record your topic
In your command prompt,
``` shell
rosbag record {your_topic_name}
```

### 2. How to plot your data (plot.py)
In this script, two messages are used for plot. Both type are _**'std_msgs/Float64'**_ and _**'nav_msgs/Odometry'**_. <br/> 
Go to _**'/launch'**_ Folder. There are three parameters you can modify.

| variable | description | 
|-----|----|
| `rosbag_path_dir` | Get rosbag file in folder '/rosbag'. | 
| `img_path_dir` | Save plot image in folder '/img'. | 
| `rosbag_name` | Fetch rosbag with name (Please modify to your actual rosbag name). | 
