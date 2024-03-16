# uga-rob-midterm
Midterm Project for CSCI 4530

Group Members:
  Diego Santamaria-Sarcos
  Marius Baden

Robot Summary:
  Our robot uses two nodes, one for navigation/movement and the other for avoiding obstacles, in addition to a helper node to help
  facillitate communication between the navigation and avoidance. The helper node uses a TimeSynchronizer to publish a laser scan 
  result and the current distance to the destination at once. To to this, we used [4].
  
  The navigation node uses subscribes to the robot's odometry (/odometry/filtered) to get the robots current pose.
  The node then determines the current direction towards the goal in the robot's frame. More specifically, it calcucates in which
  of the 12 direction classes (also called regions) the goal lies in. 
  
  Next, the navigation node passes this goal direction to the avoidance node. The avoidance node subscribes to the front laser scanner
  of the husky. Using the laser scanner, it keeps track of the obstacles in the robot's environment. The way it does it is using an 
  algorithm that we adapted from Rad-Hi [3].
  
  It iterates over the 9 direction classes (left_Left, left_Center, left_Right, front_Left,
  front_Center, front_Right, right_Left, right_Center, right_Left) and determines the closest point on the laser scan that it 
  detects per direction class. It considers this distance as the distance to the next obstacle in this direction. Taking this 
  distance to the next obstacle into account, the avoidance node calculates a cost for every direction based on how far the next
  obstacle is away (Avoider.obstacle_distance_cost_function()). This idea came from [3] but we wrote the code for it on our own to make
  it more understandable. As our own idea, we added the following additional feature: We let the obstacle cost of each direction 
  influence the obstacle cost of its neighbooring direction to the right and to the left (spread_obstacle_cost in Avoider._chose_direction()).
  In this way, we penalize directions, that would make the robot pass by close to an obstacle even though the direction 
  would be free. We do that, to avoid coliding with obstacles due to the robot's width.

  Also exceeding the approach in [3], we add a cost to each direction based on how different the direction is from the direction
  to the goal (Avoider.direction_cost_function()). The direction to the goal is the direction class that the navigation 
  node calculated. Consequently, we make the robot eventually go to the goal because going in the goal's direction
  is cheaper than driving away from it. There is also some extra overhead to handle scenarios
  in which the goal is near an obstacle by filtering out all obstacles that are further away than the obstacle.
  
  Finally, the avoidance node determines the direction with minimal combined cost of obstacles distance and direction change 
  (combined_cost_function). It returns that direction and its cost to the navigation node. We call returns the solution.

  The navigation node receives the solution and uses a proportional Controller to drive into the solution's direction. The node uses
  formulas taken from [1] and [2] to realize the proportional and to determine the current pose. We did come up with our own functions
  for slowing down the linear velocity based on the current angular velocity (in nav_husky.doControlStep()). Furthermore, we introduced
  our own formula to slow down the linear velocity the closer the robot apporaches the goal and finally stop in the goal location.

Usage:
  Launch with default goal (3.0, 1.5):
    $roslaunch rob_midterm_dsmb husky_start_to_goal.launch

  After the husky has reached its destination:
    $rosparam set rob_midterm_dsmb/destination_x -- <x>; rosparam set rob_midterm_dsmb/destination_y -- <y>
    e.g.
      $rosparam set /rob_midterm_dsmb/destination_x -- 5; rosparam set /rob_midterm_dsmb/destination_y -- -3

Contribution:
We worked on this side-by-side for the majority of the project, so the work wasn't split neatly enough to keep track of.
This is the simple summary of what we did:
  Together:
    - Planning and deciding on algorithms
    - Debugging and testing
    - README.txt
  Diego:
    - Set and updated the CMakeLists and package files
    - Started the avoidance_husky node and Avoider class [3] and modified them to support a variable goal direction from a topic
    - Created the SmartScan.msg and Solution_direction.msg message types
    - Created the merge_scans node to publish SmartScan messages to the scan topic
    - Modified nav_husky node to publish the distance to obj_distance topic
    - Created and implemeented the husky_start_to_goal launch file
  Marius:
    - Created and implemented the nav_husky node
    - Optimized the Avoider class' functions and tailored them to better fit our problem
    - Cleaned up the merge_scans node
    - Modified launch file to support parameters for the goal and husky's speed

The general paradigm of each member's contribution to the project is more easily split by roles which each member primarily
devoted their work towards:
  Diego:
    Algorithm design and sketch implementation
  Marius:
    Algorithm implementation and optimization
  
# Note from Diego - Most of my work would end up getting modified and optimized by Marius to fit the nav_husky's needs; the summary
                    lists what I did that made it to the final version, package support and overhead

Sources:
[1] https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/blob/master/gazebo-tutorial/scripts/nre_simhuskycontrol.py
[2] https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/wiki/Sec.-2:-Driving-the-Husky-robot-in-Gazebo
[3] https://github.com/Rad-hi/Obstacle-Avoidance-ROS
[4] http://wiki.ros.org/message_filters