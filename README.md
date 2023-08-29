# Citylab_project

## Introduction
<p>This project aims to exploit ROS2 architecture including topic, service, and action to control Turtlebot3 traversing a racetrack</p>

## Control Turtlebot3 in simulation via ROS topic
<p>The robot subscribes to the scanner topic and publishes the velocity topic. With these two topics, the robot is able to avoid the obstacles.<br>
To launch the code, run the following command</p>

    ros2 launch robot_patrol start_patrolling.launch.py

[![checkpoint3](https://res.cloudinary.com/marcomontalbano/image/upload/v1693272078/video_to_markdown/images/google-drive--1DGZdfjk02qPHQJp7-O23J5epePi0Vz9i-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://drive.google.com/file/d/1DGZdfjk02qPHQJp7-O23J5epePi0Vz9i/view?usp=sharing "checkpoint3")

## Control Turtlebot3 in simulation via ROS service
<p>In this section, a service server is to make decisions based on the scanner data whether to direct the robot to move forward, left, or right. A service client receives any of those commands from the server and publishes the velocity topic to move the robot.<br>
To launch the code, run the following command. This command will launch two nodes- a service server and a service client.</p>

    ros2 launch robot_patrol main.launch.py

[![checkpoint3_1](https://res.cloudinary.com/marcomontalbano/image/upload/v1693272454/video_to_markdown/images/google-drive--1Pnlb1dBlBgV25DNDlrWT__AUS5DjRJ3E-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://drive.google.com/file/d/1Pnlb1dBlBgV25DNDlrWT__AUS5DjRJ3E/view?usp=sharing "checkpoint3_1")

## Control Turtlebot3 in simulation via ROS action
<p>In this section, an action server is created to direct the robot to a goal position received via the terminal. The server will do three things.</p>
<ol>
<li>receive a goal position</li>
<li>publish feedbacks</li>
<li>send a result to a client</li>  
</ol>
<p>To launch the action server, run the following command</p>

    ros2 launch robot_patrol start_gotopoint_action.launch.py

<p>To send a goal to the server, type the following command in the terminal. For example, send a goal x=0.7, y=0.3, theta=0.0</p>

    ros2 action send_goal -f /go_to_point robot_patrol_interface/action/GoToPoint "{goal_pos: {x: 0.7, y: 0.3, z: 0.0}}"

[![checkpoint3_2](https://res.cloudinary.com/marcomontalbano/image/upload/v1693273365/video_to_markdown/images/google-drive--1eCHL0YVOvhLytsv9_cY5ovnWBLL6ULGC-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://drive.google.com/file/d/1eCHL0YVOvhLytsv9_cY5ovnWBLL6ULGC/view?usp=sharing "checkpoint3_2")




