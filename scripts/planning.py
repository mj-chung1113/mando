#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import sqrt,pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path

class planning :

    def __init__(self):
        rospy.init_node('planning', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)
        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='/map'
        
        self.is_odom=False
        self.local_path_size=70

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('mando')
        full_path=pkg_path+'/path'+'/mando_path.txt'
        self.f=open(full_path,'r')
        lines=self.f.readlines()
        for line in lines :
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1
            self.global_path_msg.poses.append(read_pose)
        
        self.f.close()

        rate = rospy.Rate(15) # 20hz
        while not rospy.is_shutdown():
   
            if self.is_odom == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y
                min_dis=float('inf')
                current_waypoint=-1
                for i,waypoint in enumerate(self.global_path_msg.poses) :

                    distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                    if distance < min_dis :
                        min_dis=distance
                        current_waypoint=i

                
                if current_waypoint != -1 :
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for num in range(current_waypoint,current_waypoint + self.local_path_size ) :
                            tmp_pose=PoseStamped()
                            tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w=1
                            local_path_msg.poses.append(tmp_pose)
                    
                    else :
                        for num in range(current_waypoint,len(self.global_path_msg.poses) ) :
                            tmp_pose=PoseStamped()
                            tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w=1
                            local_path_msg.poses.append(tmp_pose)

                #print(x,y)
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom=True
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        

if __name__ == '__main__':
    try:
        test_track=planning()
    except rospy.ROSInterruptException:
        pass
