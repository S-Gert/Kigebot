#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped
import pid
import time

speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.pub = rospy.Publisher('/kigebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.odosub = rospy.Subscriber('odometry_publisher',String, self.odometry_callback)
        self.tofsub = rospy.Subscriber('tof_publisher', String, self.tof_callback)
        
        #CALLBACK VARIABLES:
        self.bus = SMBus(1)
        self.ododata = 0
        self.tofdata = 0
        
        #PID VARIABLES:
        self.previous_left = 0
        self.previous_right = 0
        self.last_error = 0
        self.last_correction = 0
        
        self.print_counter = 0
    
    def odometry_callback(self, data):
        self.ododata = data.data
        
    def tof_callback(self, data):
        self.tofdata = data.data
        
    def publish_pid_values_to_speed(self, max_speed, correction, line_sens):
        speed.vel_left = max_speed - correction
        speed.vel_right = max_speed + correction
        if len(line_sens) == 0:
            speed.vel_left = self.previous_left
            speed.vel_right = self.previous_right
        speed.vel_left = max(0.0, min(speed.vel_left, 0.7))
        speed.vel_right = max(0.0, min(speed.vel_right, 0.7))
        self.previous_left = speed.vel_left
        self.previous_right = speed.vel_right
        self.pub.publish(speed)
        
    def turn_sharp_right(self, line_sens):
        sharp_right_turn_sens_values = [[4,5,6,7,8], #[3,4,5,6,7,8],
                                        [5,6,7,8],
                                        [6,7,8]]
        if line_sens in sharp_right_turn_sens_values:
            speed.vel_left = 0.2
            speed.vel_right = 0
            self.pub.publish(speed)
            time.sleep(0.1)
            print("Right turn")
    
    def turn_sharp_left(self, line_sens):
        sharp_left_turn_sens_values = [[1,2,3,4,5], #[1,2,3,4,5,6],
                                       [1,2,3,4],
                                       [1,2,3]]
        if line_sens in sharp_left_turn_sens_values:
            speed.vel_left = 0
            speed.vel_right = 0.2
            self.pub.publish(speed)
            time.sleep(0.1)
            print("Left turn")
            
    def turn_when_cut_in_line(self, line_sens, max_speed):
        line_values = [[1,2,4,5],
                       [1,3,4],
                       [1,4,5],
                       [1,4],
                       [1,5,6],
                       [1,2,4],
                       [2,3,5,6]]
        if line_sens in line_values:
            print("line split")
            time.sleep(0.15)
            speed.vel_left = max_speed*0.5
            speed.vel_right = max_speed
            self.pub.publish(speed)
            time.sleep(0.75)
            print("line split done")
            
    def print_data_with_interval(self, correction):
        self.print_counter = self.print_counter + 1
        if self.print_counter == 10:
            print("---| P =", rospy.get_param("/p"),
                "|---| I =", rospy.get_param("/i"),
                "|---| D =", rospy.get_param("/d"),
                '|---| Speed =', rospy.get_param("/maxvel"),
                '|---| Vel_left =', speed.vel_left,
                '|---| Vel_right =', speed.vel_right,
                '|---| Correction =', round(correction, 3),
                "|---")
            self.print_counter = 0
            #print("Odometry: ", self.ododata)
            
    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        self.bus.close()
        
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.tofdata == "wall in progress":
                self.last_error = 0
            else:
                max_speed = float(rospy.get_param("/maxvel", 0.25))
                line_sens, correction, self.last_correction, self.last_error = pid.PidClass().pid_run(self.last_error, self.last_correction)

                self.turn_sharp_right(line_sens)
                self.turn_sharp_left(line_sens)
                
                self.turn_when_cut_in_line(line_sens, max_speed)
                
                self.publish_pid_values_to_speed(max_speed, correction, line_sens)
                self.print_data_with_interval(correction)
                
            rate.sleep()
            
if __name__ == '__main__':  
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()