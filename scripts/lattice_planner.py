#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from math import cos, sin, sqrt, pow, atan2
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Quaternion
import tf

class LatticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        # Subscriber, Publisher 선언
        rospy.Subscriber("/local_path", Path, self.local_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cluster_points", PointCloud2, self.object_callback)
        rospy.Subscriber("/lfd", Float32, self.lfd_callback)

        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)
        self.lattice_viz_pub = rospy.Publisher('/lattice_viz', Path, queue_size=1)
        
        self.is_path = False
        self.is_obj = False
        self.local_path = None
        self.lattice_path = None
        self.is_odom = False

        self.lfd = 30  # 경로 생성 끝점

        self.prev_object_points = []  # 이전 프레임의 장애물 좌표
        self.prev_object_frame_counts = []  # 각 장애물의 프레임 카운트
        self.checkObject_dis = 1.6
        self.lane_weight_distance = 2.6

        base_offset = 1.5 * 0.3 * 20
        self.lane_weight = [53, 52, 51, 50, 10, 11, 12, 13]

        offset_steps = 10
        step_size = base_offset * 2 / offset_steps

        self.lane_off_set = [
            -1*base_offset,
            -1*base_offset + step_size * 1.2,
            -1*base_offset + step_size * 2.5,
            -1*base_offset + step_size * 3.7,
            base_offset - (step_size * 3.7),
            base_offset - (step_size * 2.5),
            base_offset - (step_size * 1.2),
            base_offset,
        ]
        
        rate = rospy.Rate(30)  # 30hz 로 동작하는 코드임

        # 메인 로직 구문
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_obj:
                if self.checkObject(self.local_path, self.object_points):
                    
                    lattice_path = self.latticePlanner(self.local_path, self.odom_msg)
                    lattice_path_index = self.collision_check(self.object_points, lattice_path)
                    selected_lattice_path = lattice_path[lattice_path_index]

                    # lattice_path를 상대좌표로 변환 및 퍼블리시
                    relative_lattice_path = self.convert_to_relative(selected_lattice_path)
                    self.lattice_viz_pub.publish(relative_lattice_path)

                    # 기존 절대좌표 lattice_path 퍼블리시
                    
                    self.lattice_path_pub.publish(selected_lattice_path)
                else:
                    if self.return_check(): #복귀하는 길에 전방 체크 
                        lattice_path = self.latticePlanner(self.local_path, self.odom_msg)
                        lattice_path_index = self.return_collision_check(self.object_points, lattice_path)
                        selected_lattice_path = lattice_path[lattice_path_index]

                        # lattice_path를 상대좌표로 변환 및 퍼블리시
                        relative_lattice_path = self.convert_to_relative(selected_lattice_path)
                        self.lattice_viz_pub.publish(relative_lattice_path)

                        # 기존 절대좌표 lattice_path 퍼블리시
                        self.lattice_path_pub.publish(selected_lattice_path)
                    else:
                        #local path pub 
                        self.lattice_path_pub.publish(self.local_path)
                        relative_lattice_path = self.convert_to_relative(self.local_path)
                        self.lattice_viz_pub.publish(relative_lattice_path)

            rate.sleep()

    def convert_to_relative(self, lattice_path):
        relative_path = Path()
        relative_path.header.frame_id = '/map'

        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y

        orientation = self.odom_msg.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

        for pose in lattice_path.poses:
            abs_x = pose.pose.position.x
            abs_y = pose.pose.position.y

            rel_x = cos(heading) * (abs_x - ego_x) + sin(heading) * (abs_y - ego_y)
            rel_y = -sin(heading) * (abs_x - ego_x) + cos(heading) * (abs_y - ego_y)

            relative_pose = PoseStamped()
            relative_pose.pose.position.x = rel_x
            relative_pose.pose.position.y = rel_y
            relative_pose.pose.position.z = pose.pose.position.z  # z 좌표는 그대로 유지
            relative_pose.pose.orientation = pose.pose.orientation  # 오리엔테이션 그대로 복사

            relative_path.poses.append(relative_pose)

        return relative_path

    def checkObject(self, ref_path, object_points):
        is_crash = False
        for point in object_points:
            for path in ref_path.poses:
                dis = sqrt(pow(path.pose.position.x - point[0], 2) + pow(path.pose.position.y - point[1], 2))
                if dis < self.checkObject_dis:
                    is_crash = True
                    break
        return is_crash

    def collision_check(self, object_points, out_path):
        selected_lane = 12
        self.lane_weight = [34, 33, 32, 31, 1, 2, 3, 4]
        max_weight = 10000
        for point in object_points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point[0] - path_pos.pose.position.x, 2) + pow(point[1] - path_pos.pose.position.y, 2))

                    if 3.4 < dis < 3.8:
                        self.lane_weight[path_num] += 1
                    elif 3.0 < dis < 3.4:
                        self.lane_weight[path_num] += 2
                    elif 2.7 < dis < 3.0:
                        self.lane_weight[path_num] += 3
                    elif 2.4 < dis < 2.7:
                        self.lane_weight[path_num] += 4
                    elif 2.1 < dis < 2.4:
                        self.lane_weight[path_num] += 5
                    elif 1.8 < dis < 2.1:
                        self.lane_weight[path_num] += 6
                    elif 1.5 < dis < 1.8:
                        self.lane_weight[path_num] += 7
                    elif 1.25 < dis < 1.5:
                        self.lane_weight[path_num] += 10
                    elif 1.0 < dis < 1.25:
                        self.lane_weight[path_num] += 14
                    elif dis < 1.0:
                        self.lane_weight[path_num] += 25
                    #else:
                    #    self.lane_weight[path_num] -= 1
                    self.lane_weight[path_num] = min(max_weight, self.lane_weight[path_num])

        selected_lane = self.lane_weight.index(min(self.lane_weight))
        return selected_lane
    
    def return_collision_check(self, object_points, out_path):
        selected_lane = 12
        self.lane_weight = [60, 45, 30, 10, 10, 30, 45, 60]
        max_weight = 10000
        for point in object_points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point[0] - path_pos.pose.position.x, 2) + pow(point[1] - path_pos.pose.position.y, 2))

                    if 3.4 < dis < 3.8:
                        self.lane_weight[path_num] += 1
                    elif 3.0 < dis < 3.4:
                        self.lane_weight[path_num] += 2
                    elif 2.7 < dis < 3.0:
                        self.lane_weight[path_num] += 3
                    elif 2.4 < dis < 2.7:
                        self.lane_weight[path_num] += 4
                    elif 2.1 < dis < 2.4:
                        self.lane_weight[path_num] += 5
                    elif 1.8 < dis < 2.1:
                        self.lane_weight[path_num] += 6
                    elif 1.5 < dis < 1.8:
                        self.lane_weight[path_num] += 7
                    elif 1.25 < dis < 1.5:
                        self.lane_weight[path_num] += 10
                    elif 1.0 < dis < 1.25:
                        self.lane_weight[path_num] += 14
                    elif dis < 1.0:
                        self.lane_weight[path_num] += 25
                    #else:
                    #    self.lane_weight[path_num] -= 1
                    self.lane_weight[path_num] = min(max_weight, self.lane_weight[path_num])

        selected_lane = self.lane_weight.index(min(self.lane_weight))
        return selected_lane
    
    def return_check(self):
        """
        차량의 헤딩과 현재 목표하고 있는 첫 번째 점을 기반으로 장애물이 있는지 확인.
        장애물이 부채꼴 영역(반지름 10m, 각도 제한) 내에 있으면 True 반환, 없으면 False 반환.
        """
        if not self.is_odom or not self.is_path or not self.is_obj:
            rospy.logwarn("Odom, path, or object data not received yet.")
            return False

        # 차량의 현재 위치와 헤딩 방향
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y

        orientation = self.odom_msg.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

        detection_radius = 8  # 부채꼴의 반지름 (장애물을 탐지할 거리)
        detection_angle_limit = np.pi / 2  # 부채꼴의 각도 제한 (60도)

        # 목표하는 첫 번째 점의 좌표
        if len(self.local_path.poses) == 0:
            rospy.logwarn("Local path is empty.")
            return False

        target_x = self.local_path.poses[5].pose.position.x
        target_y = self.local_path.poses[5].pose.position.y

        # 차량과 목표 점을 연결하는 선분의 각도
        angle_to_target = atan2(target_y - ego_y, target_x - ego_x)

        # 차량 헤딩과 목표 점을 연결하는 직선 간의 각도 차이
        angle_diff = abs(angle_to_target - heading)

        # 각도 차이를 0 ~ pi 범위로 정규화
        if angle_diff > np.pi:
            angle_diff = 2 * np.pi - angle_diff

        # 목표 점과 차량 헤딩 간의 각도 차이가 부채꼴 각도 제한 내에 있는지 확인
        if angle_diff <= detection_angle_limit / 2:
            # 해당 목표 점과 차량을 잇는 직선과 장애물 간 거리 판단
            for point in self.object_points:
                obj_x = point[0]
                obj_y = point[1]

                # 장애물이 차량과 목표 점을 잇는 선분에 근접한지 확인 (직선 거리)
                distance_to_line = self.point_to_line_distance(ego_x, ego_y, target_x, target_y, obj_x, obj_y)

                if distance_to_line < 1.0 and sqrt(pow(obj_x - ego_x, 2) + pow(obj_y - ego_y, 2)) < detection_radius:
                    rospy.loginfo(f"Obstacle detected near path. Distance to path: {distance_to_line}")
                    return True

        rospy.loginfo("No obstacle detected near target point.")
        return False

    def point_to_line_distance(self, x1, y1, x2, y2, px, py):
        """
        차량과 목표 점을 연결하는 선분과 장애물 사이의 수직 거리를 계산하는 함수.
        (x1, y1): 차량의 위치
        (x2, y2): 목표 점의 위치 (첫 번째 경로점)
        (px, py): 장애물의 위치
        """
        # 선분과 점 사이의 수직 거리 계산
        numerator = abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1)
        denominator = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2))
        return numerator / denominator

    def local_path_callback(self, msg):
        self.is_path = True
        self.local_path = msg
        #rospy.loginfo("Local path received and stored. is_path = True")

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        #rospy.loginfo("odom received and stored. is_odom = True")

    def object_callback(self, msg):
        if not self.is_odom:
            rospy.logwarn("odom not received yet, skipping object processing.")
            return

        self.is_obj = True
        self.object_points = []
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y
        min_distance = float('inf')

        current_object_points = []

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            abs_x, abs_y, abs_z = self.relative_to_absolute(point[0], point[1], point[2])
            current_object_points.append([abs_x, abs_y, abs_z])

        # 동적 장애물 필터링
        static_objects = self.filter_dynamic_objects(current_object_points)

        # 업데이트 후 필터링된 정적 장애물만 저장
        self.object_points = static_objects

        rospy.loginfo(f"Received and stored {len(self.object_points)} static obstacle points.")

    def filter_dynamic_objects(self, current_points):
        """
        8프레임 중 3프레임에서 장애물이 발견되면 정적 장애물로 간주.
        """
        static_objects = []
        threshold_distance = 0.8  # 동적/정적 객체를 구분할 거리 기준
        required_frames = 3  # 정적 장애물로 간주하기 위한 최소 프레임 수
        max_track_frames = 8  # 추적할 최대 프레임 수

        # 첫 프레임 처리: 이전 장애물 포인트가 없으면 현재 장애물로 초기화
        if len(self.prev_object_points) == 0:
            self.prev_object_points = current_points
            self.prev_object_frame_counts = [0] * len(current_points)
            rospy.loginfo(f"@@@@@@@@@@@@@@@@@warn@@@@@@@@@@@@@@@@@@@")
            return current_points

        # 기존의 프레임 갯수 리스트가 현재 장애물 포인트 수와 맞지 않을 경우 조정
        if len(self.prev_object_frame_counts) != len(self.prev_object_points):
            self.prev_object_frame_counts = [0] * len(self.prev_object_points)

        # 이전 프레임의 장애물과 비교하여 정적/동적 객체 판단
        for current_point in current_points:
            is_static = False
            found_match = False
            for j, prev_point in enumerate(self.prev_object_points):
                distance = sqrt(pow(current_point[0] - prev_point[0], 2) + pow(current_point[1] - prev_point[1], 2))

                if distance < threshold_distance:
                    # 일정 거리 내에서 움직이지 않으면 발견된 프레임 카운트를 증가
                    self.prev_object_frame_counts[j] = min(self.prev_object_frame_counts[j] + 1, max_track_frames)
                    found_match = True

                    # 정적 장애물로 간주할 조건 (8프레임 중 3프레임 이상 발견)
                    if self.prev_object_frame_counts[j] >= required_frames:
                        is_static = True
                        static_objects.append(current_point)
                    break

            # 동적 장애물인 경우 또는 새로운 장애물인 경우, 새로운 카운트 추가
            if not found_match:
                self.prev_object_points.append(current_point)
                self.prev_object_frame_counts.append(1 if is_static else 0)

        # 너무 오래 추적된 객체는 리스트에서 제거 (최대 프레임 수 초과한 경우)
        self.prev_object_points = [
            point for i, point in enumerate(self.prev_object_points)
            if self.prev_object_frame_counts[i] < max_track_frames
        ]
        self.prev_object_frame_counts = [
            count for count in self.prev_object_frame_counts
            if count < max_track_frames
        ]

        return static_objects

    def lfd_callback(self, msg):
        self.lfd = int(msg.data)

    def relative_to_absolute(self, rel_x, rel_y, rel_z):
        """
        장애물의 상대 좌표를 절대 좌표로 변환합니다.
        """
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y
        ego_z = self.odom_msg.pose.pose.position.z
        
        orientation = self.odom_msg.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

        abs_x = ego_x + cos(heading) * (rel_x+1.33) - sin(heading) * rel_y
        abs_y = ego_y + sin(heading) * (rel_x+1.33) + cos(heading) * rel_y
        abs_z = ego_z + rel_z  # z는 일반적으로 사용되지 않지만 포함

        return abs_x, abs_y, abs_z

    def latticePlanner(self, ref_path, vehicle_status):
        out_path = []

        orientation = vehicle_status.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))
        wheel_base = -3  # 차량의 wheel base (m)

        start_pos = {'x': vehicle_status.pose.pose.position.x, 'y': vehicle_status.pose.pose.position.y}

        if self.local_path and len(self.local_path.poses) > 0:
            last_pose = self.local_path.poses[int(min(max(3.0 * self.lfd, 20.0), 50.0))]
            end_pos = {'x': last_pose.pose.position.x, 'y': last_pose.pose.position.y}
        else:
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

        local_lattice_points = []
        for i in range(len(self.lane_off_set)):
            local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + self.lane_off_set[i], 1])

        for end_point in local_lattice_points:
            lattice_path = Path()
            lattice_path.header.frame_id = '/map'
            x = []
            y = []
            x_interval = 0.25
            xs = 0
            xf = end_point[0]
            ps = local_ego_vehicle_position[1][0]
            pf = end_point[1]
            x_num = xf / x_interval

            for i in range(xs, int(x_num)):
                x.append(i * x_interval)

            a = [0.0, 0.0, 0.0, 0.0]
            a[0] = ps
            a[1] = 0
            a[2] = 3.7 * (pf - ps) / (xf * xf)
            a[3] = -2.7 * (pf - ps) / (xf * xf * xf)

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

if __name__ == '__main__':
    try:
        LatticePlanner()
    except rospy.ROSInterruptException:
        pass
    
