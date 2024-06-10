#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

class UrGripperFakeDriver:

    def __init__(self):

        rospy.Subscriber('gripper_controller/set_pos', Float32, self.setpoint_callaback, queue_size=1) # Receives the command position
        self.state_pub = rospy.Publisher('gripper_controller/get_pos', Float32, queue_size=1) # Publishes the current position

        self.left_finger_state = Float32()
        self.left_finger_setpoint = Float32()

        self.left_finger_state.data = 0.0
        self.left_finger_setpoint.data = 0.0

        self.position = 0.0

        self.last_time = rospy.get_time()

    def setpoint_callaback(self, msg):

        self.left_finger_setpoint.data = msg.data

    def fake_gripper_control(self, setpoint):
        
        # data received in mm [0, 25]
        # data sent in mm [0 50]
        self.position = setpoint

        # if rospy.get_time() - self.last_time > 50/1e3: # Each period in ms

        #     if self.position > setpoint:
        #         self.position = self.position - 1

        #     elif self.position < setpoint:
        #         self.position = self.position + 1
            
        #     else:
        #         self.position = self.position

        #     self.last_time = rospy.get_time()


        return self.position


    def run(self):

        self.left_finger_state.data = self.fake_gripper_control(self.left_finger_setpoint.data)
        self.state_pub.publish(self.left_finger_state)

def main():

    rospy.init_node("ur_gripper_fake_driver_node")
    ur_gripper_fake_driver = UrGripperFakeDriver()
    r = rospy.Rate(50)

    while not rospy.is_shutdown():

        ur_gripper_fake_driver.run() 
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass