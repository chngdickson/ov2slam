#!/usr/bin/env python3
import rospy
import torch
from sensor_msgs.msg import Image

img_pub = rospy.Publisher('/cam0/image_raw', Image, queue_size=10)
I = 0
def imageCallback(img_data):
    global I
    if I % 2:
        pub_data = Image()
        pub_data.data = img_data.data
        pub_data.header = img_data.header
        pub_data.height = img_data.height
        pub_data.width = img_data.width
        pub_data.encoding = img_data.encoding
        pub_data.is_bigendian = img_data.is_bigendian
        pub_data.step = img_data.step
        img_pub.publish(pub_data)
    I=I+1
    
def talker():
    rospy.init_node('camera_listener')
    rospy.Subscriber("/carla/ego_vehicle/rgb_back_right/image", Image, imageCallback)
    rospy.spin()

if __name__ == '__main__':
    print(torch.cuda.is_available())
    try:
        talker()
    except rospy.ROSInterruptException:
        pass