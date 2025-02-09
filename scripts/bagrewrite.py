#!/usr/bin/env python
import rosbag
import rospy
from copy import deepcopy
import tf
import sys


if __name__=="__main__":
    if len(sys.argv) < 3:
        print("usage: my_node.py input.bag output.bag")
    
    bagInName = str(sys.argv[1])
    bagIn = rosbag.Bag(bagInName)
    bagOutName = str(sys.argv[2])
    bagOut = rosbag.Bag(bagOutName,'w')
    with bagOut as outbag:
        for topic, msg, t in bagIn.read_messages():
            if topic == '/tf':
                new_msg = deepcopy(msg)
                for i,m in enumerate(msg.transforms): # go through each frame->frame tf within the msg.transforms
                    if m.header.frame_id == "/map":
                        m.header.frame_id = "/world"
                        new_msg.transforms[i] = m
                    if m.child_frame_id == "/map":
                        m.child_frame_id = "/world"
                        new_msg.transforms[i] = m

                outbag.write(topic, new_msg, t)
            else:
                outbag.write(topic, msg, t)

    bagIn.close()
    bagOut.close()