#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


left_pub = rospy.Publisher('/geekbot/left_wheel', Int32, queue_size=10)
right_pub = rospy.Publisher('/geekbot/right_wheel', Int32, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " IR: %s", data.data)
    if data.data < 15:
        print("Driving forward.")
        msg = Int32()
        msg.data = 50
        left_pub.publish(msg)
        right_pub.publish(msg)
    else: 
        print("Too close! Stop!")
        msg = Int32()
        msg.data = 0
        left_pub.publish(msg)
        right_pub.publish(msg)

def listener():
    rospy.init_node('drive_node', anonymous=True)

    rospy.Subscriber("/geekbot/ir_cm", Int32, callback)


    rospy.spin()

if __name__ == '__main__':
    listener()
