# motion_planner
ROS package 3D motion planner for UAVs VToL.
Plan a new trajectory specifying initial and final position using a geometry_msgs::Pose type. Trajectory constraints are used to ensure a maximum velocity/acceleration/jerk.

##### Get the motion planner
To use the motion_planner package in your ROS system just clone it in your src directory:

    $ git clone https://github.com/jocacace/motion_planner.git
    
And compile the catkin workspace:

    $ roscd && cd ..
    $ catkin_make

##### Launch the motion planner
To start the motion planner package you can use the launch file placed into the package directory:

     $ roslaunch motion_planner motion_planner.launch

You can also use this file to set the desired parameters:
* *V_max*: maximum linear trajectory velocity [m/s]
* *A_max*: maximum linear acceleration velocity [m/s^2]
* *J_max*: maximum linear jerk velocity [m/s^3] 
* *aV_max*: maximum angular trajectory velocity [rad/s]
* *aA_max*: maximum angular acceleration velocity [rad/s^2]
* *aJ_max*: maximum angular jerk velocity [rad/s^3]

##### Require a new motion plan

This package servers planning requests with a standard ROS service called */generate_tarjectory*:
* Service request:  
  * geometry_msgs/Pose p_i: starting point of the trajectory
  * geometry_msgs/Pose p_f: final point of the trajectory
* Service response:
  * trajectory_msgs/JointTrajectory traj: the generated trajectory containing position, velocity and acceleration for x, y, z and yaw
