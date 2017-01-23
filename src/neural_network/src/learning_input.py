#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image


def learning_input():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub2 = rospy.Publisher('learning_input',Image, queue_size=10)
    rospy.init_node('learning_input', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz

    left_image = Image()
    left_image.header.stamp = rospy.Time.now()
    left_image.height = 3
    left_image.width = 3
    left_image.data = [255,0,0,255,0,0,255,0,0]

    middle_image = Image()
    middle_image.header.stamp = rospy.Time.now()
    middle_image.height = 3
    middle_image.width = 3
    middle_image.data = [0,255,0,0,255,0,0,255,0]

    right_image = Image()
    right_image.header.stamp = rospy.Time.now()
    right_image.height = 3
    right_image.width = 3
    right_image.data = [255,0,0,255,0,0,255,0,0]


    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)


        pub2.publish(left_image)

    rate.sleep()



if __name__ == '__main__':
    try:
        learning_input()
    except rospy.ROSInterruptException:
        pass
