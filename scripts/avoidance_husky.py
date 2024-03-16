#!/usr/bin/env python3
# Author : Diego Santamaria-sarcos
# Author : Marius Baden
# Source [3]: https://github.com/Rad-hi/Obstacle-Avoidance-ROS

from rob_midterm_dsmb.Avoider import Avoider
from rob_midterm_dsmb.msg import Solution_direction

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rob_midterm_dsmb.msg import Solution_direction
from rob_midterm_dsmb.msg import SmartScan

vel = Twist()
avoider = Avoider(vel)

def avoid_and_update(msg):
    global avoider, vel, direction_publisher
    if avoider.has_received_first_scan() == False:
        rospy.loginfo("Update not possilbe. Avoidance node has not received first scan yet.")
        return

    solution_message = avoider.find_solution_direction(msg.data)
    direction_publisher.publish(solution_message)

def main():
    global direction_publisher
    rospy.init_node('avoidance_husky')
    rospy.Subscriber("/scan", SmartScan, avoider.update_regions_with_new_scan)
    rospy.Subscriber("/nav/goal_direction", String, avoid_and_update)
    direction_publisher = rospy.Publisher("/nav/solution_direction", Solution_direction, queue_size=1)
    rate = rospy.Rate(10)
    
    rospy.loginfo("Avoidance node started")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass