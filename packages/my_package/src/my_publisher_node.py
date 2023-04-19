#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped
import pid

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
        speed.vel_left = max(0.0, min(speed.vel_left, 0.5))
        speed.vel_right = max(0.0, min(speed.vel_right, 0.5))
        self.previous_left = speed.vel_left
        self.previous_right = speed.vel_right
        self.pub.publish(speed)
        
    def turn_sharp_right(self, line_sens, max_speed):
        sharp_right_turn_sens_values = [[0,0,1,1,1,1,1,1],
                                        [0,0,0,1,1,1,1,1],
                                        [0,0,0,0,1,1,1,1],
                                        [0,0,0,0,0,1,1,1]]
        if line_sens in sharp_right_turn_sens_values:
            speed.vel_left = max_speed*2
            speed.vel_right = 0
            self.pub.publish(speed)
    
    def turn_sharp_left(self, line_sens, max_speed):
        sharp_left_turn_sens_values = [[1,1,1,1,1,1,0,0],
                                        [1,1,1,1,1,0,0,0],
                                        [1,1,1,1,0,0,0,0],
                                        [1,1,1,0,0,0,0,0]]
        if line_sens in sharp_left_turn_sens_values:
            speed.vel_left = 0
            speed.vel_right = max_speed*2
            self.pub.publish(speed)
        
    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        self.bus.close()
        
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.tofdata == "wall in progress":
                pass
            else:
                max_speed = float(rospy.get_param("/maxvel"))
                line_sens, correction, error= pid.PidClass().pid_run(self.last_error)
                
                self.turn_sharp_right(line_sens, max_speed)
                self.turn_sharp_left(line_sens, max_speed)
                
                if correction < -1 or correction > 1:
                    correction = self.last_correction
                else:
                    self.last_error = error
                    self.last_correction = correction
                
                self.publish_pid_values_to_speed(max_speed, correction, line_sens)
                
                print("---| P =", rospy.get_param("/p"),
                      "|---| I =", rospy.get_param("/i"),
                      "|---| D =", rospy.get_param("/d"),
                      '|---| Speed =', rospy.get_param("/maxvel"),
                      '|---| Correction =', round(correction, 3),
                      "|---")
                #print("Odometry: ", self.ododata)
            rate.sleep()
            
if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()