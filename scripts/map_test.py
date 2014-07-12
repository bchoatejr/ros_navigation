#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
from nav_msgs.msg import *
from nav_msgs.srv import *

    
def show_map(data):
    w = data.info.width
    h = data.info.height
    rospy.loginfo('width:%d', w)
    rospy.loginfo('height:%d', h)
    for i in range(h):
        a = ['0']*w
        for j in range(w):
            if data.data[i*w+j] != -1:
                a[j] = '1'
        print ''.join(a)
        
def save_map(data):
    f = open('map.txt', 'w')
    w = data.info.width
    h = data.info.height
    rospy.loginfo('width:%d', w)
    rospy.loginfo('height:%d', h)
    for i in range(h):
        a = ['0']*w
        for j in range(w):
            if data.data[i*w+j] != -1:
                a[j] = '1'
        print ''.join(a)
        f.write('%s\n' % a)
        
    f.write('\n\n\n')
    for i in range(h):
        for j in range(w):
            f.write('%3.2f ' % data.data[i*w+j])  
        f.write('\n')      
        
            
#    rospy.loginfo("I heard it again %s",data.data)
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

#    rospy.Subscriber("chatter", String, callback)
    test_map_sub = rospy.Subscriber('/map', OccupancyGrid, save_map)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
