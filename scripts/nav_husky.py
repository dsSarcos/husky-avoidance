#!/usr/bin/env python
# Author : Diego Santamaria-sarcos
# Author : Marius Baden
# Source [1]: https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/blob/master/gazebo-tutorial/scripts/nre_simhuskycontrol.py
#        [2]: https://github.com/SMARTlab-Purdue/ros-tutorial-gazebo-simulation/wiki/Sec.-2:-Driving-the-Husky-robot-in-Gazebo

import sys
import rospy, tf, math
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rob_midterm_dsmb.msg import Solution_direction

direction_classes = ["front_R", "front_C", "front_L", "left_R", "left_C", "left_L", "back_R", "back_C", "back_L", "right_R", "right_C", "right_L"]
direction_angle_unit = math.pi / 6
direction_to_angle = {"front_C": 0.0, "front_L": direction_angle_unit, 
                      "left_R": 2 * direction_angle_unit, "left_C": 3 * direction_angle_unit, "left_L": 4 * direction_angle_unit,
                      "back_R": 5 * direction_angle_unit, "back_C": 6 * direction_angle_unit, "back_L": -5 * direction_angle_unit,
                      "right_R": -4 * direction_angle_unit, "right_C": -3 * direction_angle_unit, "right_L": -2 * direction_angle_unit,
                      "front_R": -direction_angle_unit
                      }

latest_pose_XYTheta = [0,0,0]
destination = Point()
destination.x = rospy.get_param('/rob_midterm_dsmb/destination_x')
destination.y = rospy.get_param('/rob_midterm_dsmb/destination_y')
solution_direction_name = None
solution_direction_angle = None
solution_cost = None
distance_to_destination = None

# pController parameters
WGAIN = 1 # Gain for the angular velocity [rad/s / rad]
VMAX = rospy.get_param('rob_midterm_dsmb/velocity_max') # Linear velocity when far away [m/s]
DIST_TRESH = 0.1 # Distance treshold [m]

def main():
    rospy.init_node('nav_husky')
    rospy.loginfo('Nav: Node initialized. Destination: (%0.2f, %0.2f)', destination.x, destination.y)

    global pub_distance, velocity_pub
    goal_direction_pub = rospy.Publisher('/nav/goal_direction', String, queue_size=1)
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_distance = rospy.Publisher("/obj_distance", Float64, queue_size=1)
    localization_sub = rospy.Subscriber('/odometry/filtered', Odometry, odometryCallback)
    solution_direction_sub = rospy.Subscriber('/nav/solution_direction', Solution_direction, updateTargetedDirection)
    
    rate = rospy.Rate(15) # 10hz
    rospy.loginfo("Nav: Started node")

    while not rospy.is_shutdown():
        # Calc desired direction
        direction_class = calcDesiredDirectionClass()
        msg = String()
        msg.data = direction_class
        goal_direction_pub.publish(msg)

        rate.sleep()

        # Drive into solution direction
        if solution_direction_name is not None and distance_to_destination is not None:
            doControlStep()
        else:
            rospy.loginfo("Nav: Cannot control with solution direction {} and distance {}".format(
                solution_direction_name, distance_to_destination))

def calcDesiredDirectionClass():

    # Get desired theta
    destination_vector = [destination.x - latest_pose_XYTheta[0], destination.y - latest_pose_XYTheta[1]]
    desired_theta = math.atan2(destination_vector[1], destination_vector[0])      # From source [1]

    needed_turn = desired_theta - latest_pose_XYTheta[2]

    # Calc which of the 12 direction classes the desired theta is in
    shift = math.pi / 4
    shifted_theta = needed_turn + shift
    desired_class_id = math.floor(shifted_theta /(2 * math.pi / 12)) % 12 
    
    rospy.loginfo("Nav:\n New Theta %0.2f;\n Needed turn %0.2f;\n Direction to goal %s;\n" 
    + "Current position (%0.2f, %0.2f);\n Destination (%0.2f, %0.2f);\n",
     latest_pose_XYTheta[2], needed_turn, direction_classes[desired_class_id],
     latest_pose_XYTheta[0], latest_pose_XYTheta[1], destination.x, destination.y)
    return direction_classes[desired_class_id]


def odometryCallback(odometry_msg):
    # Update pose 
    # From [1]
    new_position = odometry_msg.pose.pose.position
    new_orientation = odometry_msg.pose.pose.orientation
    new_orientation_angles = tf.transformations.euler_from_quaternion(( # From quaternion to Euler angles
        new_orientation.x,
        new_orientation.y,  
        new_orientation.z,
        new_orientation.w))
    new_theta = new_orientation_angles[2]

    global latest_pose_XYTheta 
    latest_pose_XYTheta = [new_position.x, new_position.y, new_theta]

    global distance_to_destination
    distance_to_destination = math.sqrt((destination.x - latest_pose_XYTheta[0])**2 + (destination.y - latest_pose_XYTheta[1])**2)
    
    # Publish the distance to the avoidance node
    distanceMessage = Float64()
    distanceMessage.data = distance_to_destination
    pub_distance.publish(distanceMessage)

    return

def doControlStep():
    msg = Twist()
    # Make Husky move at the velocity specified by the avoidance node 
    # Slow down and eventually stops if Husky approaches the destination
    # From [1]
    global distance_to_destination
    if distance_to_destination > DIST_TRESH:
        effectiveYawToDo = solution_direction_angle 
        boundEffectiveYawToDo = math.atan2(math.sin(effectiveYawToDo), math.cos(effectiveYawToDo))
        msg.angular.z = min(1 , max(-1, WGAIN*(boundEffectiveYawToDo)))
        msg.linear.x = max(max(0, min(VMAX*distance_to_destination, VMAX)), -(VMAX*(distance_to_destination-1))**2 + VMAX)

        # Slow down if turning
        msg.linear.x = msg.linear.x * min(1.1*max((1 - abs(msg.angular.z)), 0), 1)

        rospy.loginfo("Controller:\n Trying to go to %s;\n Target Yaw: %0.2f;\n Current Yaw:%0.2f;\n Angular velocity %0.2f;\n Linear velocity %0.2f;\n",
         solution_direction_name, boundEffectiveYawToDo, latest_pose_XYTheta[2], msg.angular.z, msg.linear.x)

    else:
        msg.linear.x = 0
        msg.angular.z = 0
        rospy.loginfo("Controller: #-#-# Reached destination! #-#-#")

        # Get destination from the parameter server again in case it has been updated
        destination.x = rospy.get_param('/rob_midterm_dsmb/destination_x')
        destination.y = rospy.get_param('/rob_midterm_dsmb/destination_y')
    
    velocity_pub.publish(msg)
    return 

def updateTargetedDirection(solution_msg):
    global solution_direction_name, solution_direction_angle, solution_cost
    solution_direction_name = solution_msg.direction
    solution_direction_angle = direction_to_angle[solution_direction_name]
    solution_cost = solution_msg.cost

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass