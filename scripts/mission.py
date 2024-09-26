#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from mando.msg import mission  # 사용자 정의 메시지 임포트
import math

class MissionNode:
    def __init__(self):
        rospy.init_node('mission_node', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.mission_pub = rospy.Publisher('/mission', mission, queue_size=1)

        self.current_position = Point()
        self.mission_info = mission()
        
        self.proximity_threshold = 3.0

        self.previous_mission_num = 0 # 이전 미션 번호 저장
        self.count_timer = None  # 카운트를 증가시키는 타이머
        self.count_duration = 0  # 현재 카운트 시간 저장

        # 미션 0 시작 
        # 미션 1 직선주로 시작 
        # 미션 2 신호등 앞 
        # 미션 3 신호등 끝 
        # 미션 4 우회전 직전 
        # 미션 5 우회전 끝       
        # 미션 6 신호등 2 직전 
        # 미션 7 신호등 2 끝 
        # 미션 8 우회전 직전 
        # 미션 9 우회전 끝
        # 미션 10 정지 직전 
        # 미션 11 정지(END)

        # Define missions with coordinates
        self.missions = [
            {'mission_number': 0, 'x': 747.3916016125586, 'y': -547.180604854133, 'def_speed' : 40},
            {'mission_number': 1, 'x': 704.1329346221173, 'y': -556.9653338571079, 'def_speed' : 40},
            {'mission_number': 2, 'x': 534.1435547393048, 'y': -425.1507891775109, 'def_speed' : 10},
            {'mission_number': 3, 'x': 522.4193726097001, 'y': -416.7451800466515, 'def_speed' : 40},
            {'mission_number': 4, 'x': 272.35897832305636, 'y': -220.1035174508579, 'def_speed' : 20},
            {'mission_number': 5, 'x': 270.2570496122935, 'y': -180.44499389035627, 'def_speed' : 40},
            {'mission_number': 6, 'x': 328.6751709495438, 'y': -109.87042419053614, 'def_speed' : 10},
            {'mission_number': 7, 'x': 340.4031982940505, 'y': -92.36979857739061, 'def_speed' : 40},
            {'mission_number': 8, 'x': 413.37670903489925, 'y': -7.062285822350532, 'def_speed' : 10},
            {'mission_number': 9, 'x': 443.4772034197813, 'y': 3.4307762002572417, 'def_speed' : 40},
            {'mission_number': 10, 'x': 667.4838867701474, 'y': -128.98658935027197, 'def_speed' : 10},
            {'mission_number': 11, 'x': 676.3426514174207, 'y': -134.50454894499853, 'def_speed' : 10}
        ]

        rate = rospy.Rate(30)  # 15hz
        while not rospy.is_shutdown():
            self.update_mission()
            self.mission_pub.publish(self.mission_info)
            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def count_timer_callback(self, event):
        """Callback function for the timer to increase the count."""
        if self.count_duration < 5:
            self.mission_info.count += 1
            self.count_duration += 1
        else:
            self.mission_info.count = 0
            self.count_timer.shutdown()  # Stop the timer after 5 seconds
            self.count_duration = 0  # Reset count duration

    def update_mission(self):
        """Update the current mission based on the current position."""
        # Iterate through missions to find the one we're currently targeting
        for mission in self.missions:
            dist = self.distance(self.current_position.x, self.current_position.y, mission['x'], mission['y'])
            
            if dist < self.proximity_threshold:
                # Mission reached, update mission info
                if mission['mission_number'] != self.previous_mission_num:
                    self.mission_info.mission_num = mission['mission_number']
                    self.mission_info.speed = mission['def_speed']
                    self.mission_info.count = 1  # Reset and start count at 1
                    
                    self.previous_mission_num = mission['mission_number']
                    
                    if self.count_timer is not None:
                        self.count_timer.shutdown()  # Stop the previous timer if it exists
                    
                    # Start a new repeating timer to increment the count every second for 5 seconds
                    self.count_duration = 1  # Initialize count duration
                    self.count_timer = rospy.Timer(rospy.Duration(1.0), self.count_timer_callback, oneshot=False)


if __name__ == '__main__':
    try:
        mission_node = MissionNode()
    except rospy.ROSInterruptException:
        pass
