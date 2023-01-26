# MathWorks' ROS and Gazebo

[Get Started with Gazebo and Simulated TurtleBot](https://www.mathworks.com/help/ros/ug/get-started-with-gazebo-and-a-simulated-turtlebot.html)

I was using MathWorks' virtual machine ros_noetic_foxy_gazebov11_linux_win_v2, but the machine ROS_IP is not well configured, so I extracted the necessnary files out so you can run in your own ROS environment.

Just for the world files have to download whole virtual machine. Mathworks should release the world they used and other source codes for us ROS users.

Remeber to specify the model in .bashrc:

```
export TURTLEBOT3_MODEL=burger
```

If you want to stick to the virtual machine options, change ROS_IP in .bashrc and starting scripts to start ROS in virtual machine correctly. Oringinal cmd $(hostname -I | tr -d [:blank:]) in scripts and .bashrc extract some other output so cannot be used.
