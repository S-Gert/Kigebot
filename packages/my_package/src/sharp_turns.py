#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
import time
import my_publisher_node as mpn

speed = WheelsCmdStamped()

class sharpTurnClass():
    def __init__(self):
        self.pub = rospy.Publisher('/kigebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.sharp_turn = False
        self.hz_count = 0
        self.save_speed_once_var = 0
        self.prev_velocity = 0
        self.sharp_left_turn_sens_values = [[1,2,3,4,5,6],
                                            [1,2,3,4,5],
                                            [1,2,3,4],
                                            [1,2,3]]
        self.sharp_right_turn_sens_values = [[3,4,5,6,7,8],
                                             [4,5,6,7,8],
                                             [5,6,7,8],
                                             [6,7,8]]
        self.line_values = [[1,2,4,5],
                            [1,3,4],
                            [1,4,5],
                            [1,4],
                            [1,5,6],
                            [1,2,4],
                            [2,3,5,6]]
    
    def turn_90_degrees(self, line_sens, max_speed):
        if line_sens in self.sharp_left_turn_sens_values or line_sens in self.sharp_right_turn_sens_values:
            if line_sens in self.sharp_left_turn_sens_values:
                speed.vel_left = 0
                speed.vel_right = max_speed*2
                print("Left turn")
            if line_sens in self.sharp_right_turn_sens_values:
                speed.vel_left = max_speed*2
                speed.vel_right = 0
                print("Right turn")
            self.pub.publish(speed)
            self.sharp_turn = True
    
    def sharp_turn_speed_change(self, max_speed):
        if self.sharp_turn == True:
            if self.save_speed_once_var == 0:
                self.prev_velocity = max_speed
                self.save_speed_once_var = 1
            rospy.set_param("/maxvel", 0.2)
            self.hz_count += 1
        if self.hz_count > 60:
            rospy.set_param("/maxvel", self.prev_velocity)
            self.sharp_turn = False
            self.save_speed_once_var = 0
            self.hz_count = 0
            
    def turn_when_cut_in_line(self, line_sens, max_speed):
        if line_sens in self.line_values:
            print("line split")
            time.sleep(0.15)
            speed.vel_left = max_speed*0.3
            speed.vel_right = max_speed
            self.pub.publish(speed)
            time.sleep(0.75)

#     def run(self):
#         rate = rospy.Rate(20)
#         while not rospy.is_shutdown():
#             line_sens = mpn.node.line_sens
#             max_speed = mpn.node.max_speed
            
#             self.turn_90_degrees(line_sens, max_speed)
#             self.sharp_turn_speed_change(max_speed)
#             rate.sleep()

# if __name__ == '__main__':
#     inst = sharpTurnClass()
#     inst.run()
#     rospy.spin()