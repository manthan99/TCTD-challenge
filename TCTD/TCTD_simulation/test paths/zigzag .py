#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math

def frange(x, y, jump):
    while x < y:
        yield x
        x += jump

def main():
    rospy.init_node('astroid_curve_publisher')
    
    path_pub = rospy.Publisher('astroid_path', Path, queue_size=10)
    path = Path()

    path.header.frame_id = rospy.get_param('~output_frame', 'map')
    radius = rospy.get_param('~radius', 10.0)
    resolution = rospy.get_param('~resolution', 0.01)
    resolution_l=rospy.get_param('~resolution_l',1)
    holonomic = rospy.get_param('~holonomic', False)
    offset_x = rospy.get_param('~offset_x', 0.0)
    offset_y = rospy.get_param('~offset_y', 10.0)
    update_rate = rospy.get_param('~update_rate', 0.3)

    has_initialize = True
    x =100    
    y =375
    for t in frange (0,1.2,resolution):
        x = x + t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0
        # if holonomic:
        #     yaw = -math.sin(t) / math.cos(t)
        # else:
        #     if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
        #         yaw = math.atan2(old_y - y, old_x - x)
        #     else:
        #         yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
        
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    for t in frange(-math.pi/2, math.pi/2, resolution):
        x = radius * math.cos(t) + last_x
        y = radius * math.sin(t) + last_y
        
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius

    for t in frange (0,1,resolution):
        x = x - t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = -math.pi
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
      
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    print last_y,last_x
    i =0
    for t in frange(-math.pi/2, +math.pi/2, resolution):
        print "h"
        x = -radius * math.cos(-t) + last_x
        y = -radius * math.sin(-t) + last_y
        if(i==0):
            print x,y
            i=i+1
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    for t in frange (0,1,resolution):
        x = x + t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0
        # if holonomic:
        #     yaw = -math.sin(t) / math.cos(t)
        # else:
        #     if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
        #         yaw = math.atan2(old_y - y, old_x - x)
        #     else:
        #         yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
        
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    for t in frange(-math.pi/2, math.pi/2, resolution):
        x = radius * math.cos(t) + last_x
        y = radius * math.sin(t) + last_y
        
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    for t in frange (0,1,resolution):
        x = x - t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = -math.pi
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
      
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    for t in frange(-math.pi/2, +math.pi/2, resolution):
        print "h"
        x = -radius * math.cos(-t) + last_x
        y = -radius * math.sin(-t) + last_y
        if(i==0):
            print x,y
            i=i+1
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    for t in frange (0,1,resolution):
        x = x + t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0
        # if holonomic:
        #     yaw = -math.sin(t) / math.cos(t)
        # else:
        #     if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
        #         yaw = math.atan2(old_y - y, old_x - x)
        #     else:
        #         yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
        
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    for t in frange(-math.pi/2, math.pi/2, resolution):
        x = radius * math.cos(t) + last_x
        y = radius * math.sin(t) + last_y
        
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    for t in frange (0,1,resolution):
        x = x - t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = -math.pi
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
      
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    
    for t in frange(-math.pi/2, +math.pi/2, resolution):
        print "h"
        x = -radius * math.cos(-t) + last_x
        y = -radius * math.sin(-t) + last_y
        if(i==0):
            print x,y
            i=i+1
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    for t in frange (0,1,resolution):
        x = x + t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0
        # if holonomic:
        #     yaw = -math.sin(t) / math.cos(t)
        # else:
        #     if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
        #         yaw = math.atan2(old_y - y, old_x - x)
        #     else:
        #         yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
        
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    for t in frange(-math.pi/2, math.pi/2, resolution):
        x = radius * math.cos(t) + last_x
        y = radius * math.sin(t) + last_y
        
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    for t in frange (0,1,resolution):
        x = x - t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = -math.pi
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
      
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    for t in frange(-math.pi/2, +math.pi/2, resolution):
        print "h"
        x = -radius * math.cos(-t) + last_x
        y = -radius * math.sin(-t) + last_y
        if(i==0):
            print x,y
            i=i+1
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    for t in frange (0,1,resolution):
        x = x + t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0
        # if holonomic:
        #     yaw = -math.sin(t) / math.cos(t)
        # else:
        #     if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
        #         yaw = math.atan2(old_y - y, old_x - x)
        #     else:
        #         yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
        
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y+radius
    for t in frange(-math.pi/2, math.pi/2, resolution):
        x = radius * math.cos(t) + last_x
        y = radius * math.sin(t) + last_y
        
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    for t in frange (0,1.2,resolution):
        x = x - t
        # y = y + t
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = -math.pi
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
      
        old_x = x
        old_y = y
    last_x =old_x
    last_y =old_y-radius
    for t in frange(-math.pi/2,0, resolution):
        x = -radius * math.cos(t) + last_x
        y = -radius * math.sin(t) +last_y
        
        # if has_initialize:
        #     old_x = x
        #     old_y = y
        #     has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <=math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)
        
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)

        old_x = x
        old_y = y
    last_x =x
    last_y =old_y+radius
    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        path.header.stamp = rospy.get_rostime()
        path_pub.publish(path)
        #r.sleep()
    
if __name__ == '__main__':
    main()

