#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, sqrt, pow, atan2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
import sensor_msgs.point_cloud2 as pc2
#from kurrier.msg import mission  # 사용자 정의 메시지 임포트
from geometry_msgs.msg import Quaternion
import tf

class LatticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        # Subscriber, Publisher 선언
        rospy.Subscriber("/local_path", Path, self.local_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cluster_points", PointCloud2, self.object_callback)
        #rospy.Subscriber("/mission", mission, self.mission_callback)

        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)

        self.is_path = False
        self.is_obj = False
        
        self.local_path = None
        self.lattice_path = None
        self.is_odom=False

        self.index = 35  # 경로 생성 끝점

        base_offset = 1.6 * 0.3 * 20  # 인덱스 1당 30cm의 증가율 적용

        self.lane_weight = [ 4, 3, 2, 1, 1, 2, 3, 4 ]

        offset_steps = 10
        step_size = base_offset * 2 / offset_steps

        self.lane_off_set = [
            -1*base_offset,
            -1*base_offset + step_size * 1.5,
            -1*base_offset + step_size * 2.5,
            -1*base_offset + step_size * 3.5,
            base_offset - (step_size * 3.5),
            base_offset - (step_size * 2.5),
            base_offset - (step_size * 1.5),
            base_offset,
        ]

        self.checkObject_dis = 2.5
        self.lane_weight_distance = 2.6

        rate = rospy.Rate(30)  # 30hz 로 동작하는 코드임

        #메인 로직 구문 
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_obj: #콜백 함수들 돌아가고 있으면
                if self.checkObject(self.local_path, self.object_points):
                    lattice_path = self.latticePlanner(self.local_path, self.odom_msg)
                    lattice_path_index = self.collision_check(self.object_points, lattice_path)
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                else:
                    self.lattice_path_pub.publish(self.local_path)
                    
            rate.sleep()
    #local_path 위에 장애물(라이다로 감지)이 있는 지 확인하는 함수 
    def checkObject(self, ref_path, object_points):
        is_crash = False
        for point in object_points:
            for path in ref_path.poses:
                dis = sqrt(pow(path.pose.position.x - point[0], 2) + pow(path.pose.position.y - point[1], 2))
                if dis < self.checkObject_dis:  # 장애물의 좌표값이 지역 경로 상의 좌표값과의 직선거리가 self.checkObject_dis 미만일 때 충돌이라 판단
                    # dis < checkobject_dis
                    is_crash = True
                    break
        return is_crash
    #생성될 후보 lattice_path 들과 장애물과의 거리를 계산해 가중치를 부여하는 함수 
    def collision_check(self, object_points, out_path):
        selected_lane = 12
        self.lane_weight = [ 4, 3, 2, 1, 1, 2, 3, 4 ]
        max_weight = 3000
        for point in object_points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point[0] - path_pos.pose.position.x, 2) + pow(point[1] - path_pos.pose.position.y, 2))
                    
                    if  2.8 < dis < 3.5:
                        self.lane_weight[path_num] += 1
                    elif 2.4 < dis < 2.8 :
                        self.lane_weight[path_num] += 2
                    elif  2.0 < dis < 2.4:
                        self.lane_weight[path_num] += 4
                    elif  1.7 < dis < 2.0:
                        self.lane_weight[path_num] += 7
                    elif  1.5 < dis < 1.7:
                        self.lane_weight[path_num] += 10
                    elif  1.3 < dis < 1.5:
                        self.lane_weight[path_num] += 15
                    elif  1.1 < dis < 1.3:
                        self.lane_weight[path_num] += 21
                    elif  0.9 < dis < 1.1:
                        self.lane_weight[path_num] += 30
                    elif   dis < 0.9 :
                        self.lane_weight[path_num] += 50
                    else: 
                        self.lane_weight[path_num] -= 1
                    self.lane_weight[path_num] = min(max_weight, self.lane_weight[path_num])

        for i, weight in enumerate(self.lane_weight):
            rospy.loginfo(f"Lane {i} weight: {weight}")        
        selected_lane = self.lane_weight.index(min(self.lane_weight))
        return selected_lane
    #local_path 받아오는 콜백함수 
    def local_path_callback(self, msg):
        self.is_path = True
        self.local_path = msg
        rospy.loginfo("Local path received and stored. is_path = True")
    
    #gps,imu를 통해 구해진 odom (내 차량의 좌표 등등)받아오는 콜백함수 
    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        rospy.loginfo("odom received and stored. is_odom = True")

    def object_callback(self, msg):
        if not self.is_odom:
            rospy.logwarn("odom not received yet, skipping object processing.")
            return

        self.is_obj = True
        self.object_points = []
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y
        min_distance = float('inf')  # 초기화: 아주 큰 값으로 설정
        # 클러스터링된 장애물 포인트들을 추출하여 절대 좌표로 변환하여 저장합니다.
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            abs_x, abs_y, abs_z = self.relative_to_absolute(point[0], point[1], point[2])
            self.object_points.append([abs_x, abs_y, abs_z])
            
            distance = sqrt(pow(abs_x - ego_x, 2) + pow(abs_y - ego_y, 2))
            if distance < min_distance:
                min_distance = distance
        #rospy.loginfo(f"장애물과 차량 사이의 최소 거리: {min_distance}m")
        #rospy.loginfo(f"Received and stored {len(self.object_points)} obstacle points. is_obj = True")
        #if len(self.object_points) > 0:
            #rospy.loginfo(f"First obstacle point (absolute): x={self.object_points[0][0]}, y={self.object_points[0][1]}, z={self.object_points[0][2]}")
    
    #안쓰지만 나중에 쓸지도 
    def head_check(self):
        """
        클러스터된 장애물 전방 존재 여부 체크 
        """
        for point in self.object_points:
            # point[0] = x (상대좌표), point[1] = y (상대좌표)
            rel_x = point[0]
            rel_y = point[1]

            # 장애물이 차량 앞 3m < x < 8m, -2m < y < 2m 범위에 있는지 확인
            if 3 < rel_x < 10 and -2 < rel_y < 2:
                #rospy.loginfo(f"장애물 발견: 상대좌표 (x: {rel_x}, y: {rel_y}) 범위 내에 장애물이 있습니다.")
                return True

        rospy.loginfo("장애물이 지정된 범위 내에 없습니다.")
        return False
    
    #라이다로 감지된 상대좌표인 장애물 좌표를 절대좌표로 변환하는 함수 
    def relative_to_absolute(self, rel_x, rel_y, rel_z):
        """
        장애물의 상대 좌표를 절대 좌표로 변환합니다.
        """
        # 차량의 현재 위치와 헤딩 정보를 가져옵니다.
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y
        ego_z = self.odom_msg.pose.pose.position.z
        
        # 오리엔테이션에서 헤딩 방향을 계산
        orientation = self.odom_msg.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))
        
        wheel_base = -3.4  # 차량의 wheel base (m)
        front_axle_x = ego_x + cos(heading) * wheel_base
        front_axle_y = ego_y #+ sin(heading) * wheel_base   
        # 변환 행렬을 사용하여 상대 좌표를 절대 좌표로 변환
        abs_x = front_axle_x + cos(heading) * (rel_x + 0.0) - sin(heading) * rel_y
        abs_y = front_axle_y + sin(heading) * (rel_x + 0.0) + cos(heading) * rel_y
        abs_z = ego_z + rel_z  # z는 일반적으로 사용되지 않지만 포함

        #rospy.loginfo(f"Converted relative position ({rel_x}, {rel_y}, {rel_z}) to absolute position ({abs_x}, {abs_y}, {abs_z}) with heading {heading}")
        return abs_x, abs_y, abs_z

    #핵심코드, lattice_path를 생성하는 로직. 양쪽 총 8개의 후보 path를 생성하는 과정임. 
    def latticePlanner(self, ref_path, vehicle_status):
        out_path = []

        # vehicle_velocity = max(vehicle_status.twist.twist.linear.x * 3.6, 20)  # 정지 시에도 기본 속도(20)를 사용하여 look_distance 계산
        # look_distance = int(vehicle_velocity * 0.2 * 2)

        # 오리엔테이션에서 헤딩 방향을 계산
        orientation = vehicle_status.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))
        wheel_base = -1  # 차량의 wheel base (m)
     
        # 시작점과 끝점을 차량의 현재 위치 기준으로 정함
        start_pos = {'x': vehicle_status.pose.pose.position.x  , 'y': vehicle_status.pose.pose.position.y }

        # self.local_path의 마지막 점을 end_pos에 저장하는 코드
        if self.local_path and len(self.local_path.poses) > 0:
            last_pose = self.local_path.poses[self.index]
            end_pos = {'x': last_pose.pose.position.x, 'y': last_pose.pose.position.y}
            #rospy.loginfo(f"End position set to x: {end_pos['x']}, y: {end_pos['y']}")
        else:
            #rospy.logwarn("local_path is empty or not set.")
            end_pos = None

        theta = atan2(end_pos['y'] - start_pos['y'], end_pos['x'] - start_pos['x'])
        translation = [start_pos['x'], start_pos['y']]

        trans_matrix = np.array([[cos(theta), -sin(theta), translation[0]],
                                 [sin(theta), cos(theta), translation[1]],
                                 [0, 0, 1]])

        det_trans_matrix = np.array([[trans_matrix[0][0], trans_matrix[1][0], -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])],
                                     [trans_matrix[0][1], trans_matrix[1][1], -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                     [0, 0, 1]])

        world_end_point = np.array([[end_pos['x']], [end_pos['y']], [1]])
        local_end_point = det_trans_matrix.dot(world_end_point)
        world_ego_vehicle_position = np.array([[vehicle_status.pose.pose.position.x], [vehicle_status.pose.pose.position.y], [1]])
        local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
        # lane_off_set = [-2.0, -1.5, -1.0, 1.0, 1.5, 2.0]

        local_lattice_points = []

        for i in range(len(self.lane_off_set)):
            local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + self.lane_off_set[i], 1])

        for end_point in local_lattice_points:
            lattice_path = Path()
            lattice_path.header.frame_id = 'map'
            x = []
            y = []
            x_interval = 0.25 #path를 이루는 점들 사이의 간격 (커질 수록 앞(x)축 방향으로 간격이 넓어지고, 작아질수록 촘촘)
            xs = 0
            xf = end_point[0]
            ps = local_ego_vehicle_position[1][0]
            pf = end_point[1]
            x_num = xf / x_interval

            for i in range(xs, int(x_num)):
                x.append(i * x_interval)

            #path가 될 3차함수의 모양을 결정
            a = [0.0, 0.0, 0.0, 0.0]
            a[0] = ps
            a[1] = 0
            a[2] = 4.7 * (pf - ps) / (xf * xf)
            a[3] = -3.7 * (pf - ps) / (xf * xf * xf)

            for i in x:
                result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                y.append(result)

            for i in range(0, len(y)):
                local_result = np.array([[x[i]], [y[i]], [1]])
                global_result = trans_matrix.dot(local_result)

                read_pose = PoseStamped()
                read_pose.pose.position.x = global_result[0][0]
                read_pose.pose.position.y = global_result[1][0]
                read_pose.pose.position.z = 0
                read_pose.pose.orientation.x = 0
                read_pose.pose.orientation.y = 0
                read_pose.pose.orientation.z = 0
                read_pose.pose.orientation.w = 1
                lattice_path.poses.append(read_pose)

            out_path.append(lattice_path)
                       
        return out_path
    #안쓰지만 필요할지도 
    def generate_local_path(self, vehicle_status):
        """
        Generates a local path starting from (0, 0, 0) and extending straight along the x-axis.
        The path extends for 20 points with 0.3 meters between each point.
        """
        path = Path()
        path.header.frame_id = 'map'

        # The number of points (steps) in the path
        num_points = 30

        # Interval between points in meters
        interval = 0.3

        # Generate points along the x-axis
        for i in range(num_points):
            x = i * interval
            y = 0.0  # No deviation in y, so it stays on the x-axis
            z = 0.0  # Assuming a flat 2D plane

            # Create a PoseStamped for each point
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            # Set the orientation to face straight along the x-axis
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # No rotation, aligned with x-axis
            pose.pose.orientation = Quaternion(*quaternion)

            # Add the pose to the path
            path.poses.append(pose)

        return path


if __name__ == '__main__':
    try:
        LatticePlanner()
    except rospy.ROSInterruptException:
        pass
