#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from math import cos, sin, sqrt, pow, atan2
from std_msgs.msg import Float32,Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped,PointStamped
from nav_msgs.msg import Path, Odometry
import numpy as np
from mando.msg import mission 
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
        rospy.Subscriber("/mission", mission, self.mission_callback)
        
        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)
        self.lattice_viz_pub = rospy.Publisher('/lattice_viz', Path, queue_size=1)
        self.end_pose_point_pub = rospy.Publisher('/end_pose_point', PointStamped, queue_size=1)  # PointStamped 퍼블리셔 추가
        self.lattice_on_pub = rospy.Publisher('/lattice_on', Bool, queue_size=1)

        self.is_path = False
        self.is_obj = False
        self.local_path = None
        self.lattice_path = None
        self.is_odom = False
        self.is_mission_end = False


        self.lfd = 30  # 경로 생성 끝점
        self.hderr_threshold = 2.0 #lattice endpose 에러 임계 설정 
        self.prev_object_points = []  # 이전 프레임의 장애물 좌표
        self.prev_object_frame_counts = []  # 각 장애물의 프레임 카운트
        self.checkObject_dis = 1.6
        self.lane_weight_distance = 2.6

        base_offset = 1.5 * 0.3 * 20
        self.lane_weight = [53, 52, 51, 50,5,5, 10, 11, 12, 13]
        self.return_weight = [60, 45, 30, 10,5,5, 10, 30, 45, 60]
        offset_steps = 10
        step_size = base_offset * 2 / offset_steps

        self.lane_off_set = [
            -1*base_offset,
            -1*base_offset + step_size * 1.2,
            -1*base_offset + step_size * 2.4,
            -1*base_offset + step_size * 3.6,
            -1*base_offset + step_size * 4.9,
             base_offset - (step_size * 4.9),
            base_offset - (step_size * 3.6),
            base_offset - (step_size * 2.4),
            base_offset - (step_size * 1.2),
            base_offset,
        ]
        
        rate = rospy.Rate(30)  # 30hz 로 동작하는 코드임

        # 메인 로직 구문
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_obj:
                if self.mission_info.mission_num not in [2, 6, 10] and self.checkObject(self.local_path, self.object_points):
                    
                    lattice_path = self.latticePlanner(self.local_path, self.odom_msg)
                    
                    if len(lattice_path) > 0:  # lattice_path가 비어있지 않은지 확인
                        lattice_path_index = self.collision_check(self.object_points, lattice_path)
                        if 0 <= lattice_path_index < len(lattice_path):
                            selected_lattice_path = lattice_path[lattice_path_index]

                            # lattice_path를 상대좌표로 변환 및 퍼블리시
                            relative_lattice_path = self.convert_to_relative(selected_lattice_path)
                            self.lattice_viz_pub.publish(relative_lattice_path)

                            # 기존 절대좌표 lattice_path 퍼블리시
                            self.lattice_path_pub.publish(selected_lattice_path)
                            self.lattice_on_pub.publish(True)
                            rospy.loginfo("org_lattice ######################")
                        else:
                            rospy.logwarn("Invalid lattice_path_index")
                    else:
                        rospy.logwarn("lattice_path is empty")
                else:
                    if self.mission_info.mission_num not in [2, 6, 10] and self.return_check():
                        lattice_path = self.latticePlanner(self.local_path, self.odom_msg)

                        if len(lattice_path) > 0:  # lattice_path가 비어있지 않은지 확인
                            lattice_path_index = self.return_collision_check(self.object_points, lattice_path)
                            if 0 <= lattice_path_index < len(lattice_path):
                                selected_lattice_path = lattice_path[lattice_path_index]

                                # lattice_path를 상대좌표로 변환 및 퍼블리시
                                relative_lattice_path = self.convert_to_relative(selected_lattice_path)
                                self.lattice_viz_pub.publish(relative_lattice_path)
                                self.lattice_on_pub.publish(True)

                                # 기존 절대좌표 lattice_path 퍼블리시
                                self.lattice_path_pub.publish(selected_lattice_path)
                                rospy.loginfo("return_check !!!!!!!!!!!!!!!!!!!!!!!!!!")
                            else:
                                rospy.logwarn("Invalid lattice_path_index")
                        else:
                            rospy.logwarn("lattice_path is empty")
                    else:
                        # local path pub
                        self.lattice_path_pub.publish(self.local_path)
                        relative_lattice_path = self.convert_to_relative(self.local_path)
                        self.lattice_viz_pub.publish(relative_lattice_path)
                        self.lattice_on_pub.publish(False)
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
        path_length_limit = max(10, int(len(ref_path.poses) * (2.0 / 3.0)))  # 경로의 3분의 2 지점까지만 확인
        path_length_unlimit = max(5, int(len(ref_path.poses) * (1.0 / 6.0)))

        # 차량의 현재 위치
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y

        for point in object_points:
            # 경로의 3분의 2 지점까지만 확인
            for i in range(path_length_unlimit, path_length_limit):
                path = ref_path.poses[i]
                dis = sqrt(pow(path.pose.position.x - point[0], 2) + pow(path.pose.position.y - point[1], 2))

                # 장애물이 경로 근처에 있으면 충돌로 간주
                if dis < self.checkObject_dis:
                    is_crash = True
                    break

            if is_crash:
                break

        if not is_crash:
            # y 좌표 차이를 기반으로 새로운 경로 생성
            is_crash = self.generateAdjustedPathAndCheck(object_points)

        return is_crash
    
    def generateAdjustedPathAndCheck(self, object_points):
        is_crash = False
        adjusted_path = Path()
        adjusted_path.header.frame_id = '/map'

        # 차량의 현재 위치와 헤딩 계산
        ego_x = self.odom_msg.pose.pose.position.x
        ego_y = self.odom_msg.pose.pose.position.y
        orientation = self.odom_msg.pose.pose.orientation
        heading = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y**2 + orientation.z**2))

        # 차량의 현재 위치를 헤딩 방향으로 1미터 앞으로 보정
        adjusted_ego_x = ego_x + cos(heading) * 1.0
        adjusted_ego_y = ego_y + sin(heading) * 1.0

        # 로컬 경로의 첫 번째 점과 차량의 y좌표 차이 계산
        closest_point = self.local_path.poses[0].pose.position
        x_offset = closest_point.x - adjusted_ego_x
        y_offset = closest_point.y - adjusted_ego_y 

        # y_offset을 반영한 새로운 경로 생성
        for pose in self.local_path.poses:
            new_pose = PoseStamped()
            new_pose.pose.position.x = pose.pose.position.x - x_offset
            new_pose.pose.position.y = pose.pose.position.y - y_offset  # y좌표 차이만큼 경로를 이동
            new_pose.pose.position.z = pose.pose.position.z
            new_pose.pose.orientation = pose.pose.orientation
            adjusted_path.poses.append(new_pose)

        # 경로의 3분의 2 지점까지만 확인
        path_length_limit = max(10, int(len(adjusted_path.poses) * (2.0 / 3.0)))
        

        # 생성된 경로에 대해 장애물 충돌 확인
        for point in object_points:
            # 경로의 3분의 2 지점까지만 확인
            for i in range(path_length_limit):
                path_pose = adjusted_path.poses[i]
                dis = sqrt(pow(path_pose.pose.position.x - point[0], 2) + pow(path_pose.pose.position.y - point[1], 2))

                # 장애물이 경로 근처에 있으면 충돌로 간주
                if dis < 1.8:
                    is_crash = True
                    break

            if is_crash:
                break

        return is_crash

    def collision_check(self, object_points, out_path):
        selected_lane = 12
        self.lane_weight = [120, 100, 80, 60, 1, 3, 5, 10, 20, 21]  # 초기 레인별 가중치
        max_weight = 10000  # 최대 가중치

        for point in object_points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point[0] - path_pos.pose.position.x, 2) + pow(point[1] - path_pos.pose.position.y, 2))

                    if 2.0 <= dis < 2.5:
                        self.lane_weight[path_num] += 1
                    elif 1.5 <= dis < 2.0:
                        self.lane_weight[path_num] += 3
                    elif 1.25 <= dis < 1.5:
                        self.lane_weight[path_num] += 14
                    elif 1.0 <= dis < 1.25:
                        self.lane_weight[path_num] += 25
                    elif 0.5<= dis < 1.0:
                        self.lane_weight[path_num] += 37
                    elif dis < 0.5:
                        self.lane_weight[path_num] += 49
                    #else:
                    #    self.lane_weight[path_num] -= 1
                    self.lane_weight[path_num] = min(max_weight, self.lane_weight[path_num])


        # 각 레인의 가중치를 출력
        for i, weight in enumerate(self.lane_weight):
            rospy.loginfo(f"Lane {i}: Weight = {weight}")

        selected_lane = self.lane_weight.index(min(self.lane_weight))
        return selected_lane

    def return_collision_check(self, object_points, out_path):
        selected_lane = 12
        self.return_weight = [62, 45, 30, 10, 3, 5, 10, 30, 45, 60]
        max_weight = 10000
        for point in object_points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point[0] - path_pos.pose.position.x, 2) + pow(point[1] - path_pos.pose.position.y, 2))

                    if 1.25 <= dis < 1.5:
                        self.lane_weight[path_num] += 16
                    elif 1.0 <= dis < 1.25:
                        self.lane_weight[path_num] += 25
                    elif 0.5<= dis < 1.0:
                        self.lane_weight[path_num] += 37
                    elif dis < 0.5:
                        self.lane_weight[path_num] += 49
                    #else:
                    #    self.lane_weight[path_num] -= 1
                    self.lane_weight[path_num] = min(max_weight, self.lane_weight[path_num])

        selected_lane = self.lane_weight.index(min(self.lane_weight))
        return selected_lane
    
    def return_check(self):
        """
        차량의 헤딩과 현재 목표하고 있는 첫 번째 점과 로컬 경로 중간 지점을 기반으로 부채꼴을 형성.
        해당 영역 안에 장애물이 있는지 확인하여 True 또는 False 반환.
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

        if len(self.local_path.poses) == 0:
            rospy.logwarn("Local path is empty.")
            return False

        # 차량에서 가장 가까운 로컬 경로 점 찾기
        closest_index = 0
        min_dist = float('inf')
        for i, pose in enumerate(self.local_path.poses):
            dist = sqrt(pow(ego_x - pose.pose.position.x, 2) + pow(ego_y - pose.pose.position.y, 2))
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        # 가까운 로컬 포인트와 전체 경로 중간 지점
        closest_point = self.local_path.poses[closest_index].pose.position
        middle_index = int(len(self.local_path.poses) / 2)
        middle_point = self.local_path.poses[middle_index].pose.position

        # 반지름 길이: 차량과 가장 가까운 로컬 포인트까지의 거리
        radius = sqrt(pow(ego_x - closest_point.x, 2) + pow(ego_y - closest_point.y, 2))

        # 차량과 목표 점을 연결하는 선분의 각도
        angle_to_target = atan2(closest_point.y - ego_y, closest_point.x - ego_x)

        # 차량 헤딩과 목표 점을 연결하는 직선 간의 각도 차이
        angle_diff = abs(angle_to_target - heading)

        # 각도 차이를 0 ~ pi 범위로 정규화
        if angle_diff > np.pi:
            angle_diff = 2 * np.pi - angle_diff

        # 부채꼴의 각도 제한 (60도)
        detection_angle_limit = np.pi   # 60도

        # 목표 점과 차량 헤딩 간의 각도 차이가 부채꼴 각도 제한 내에 있는지 확인
        if angle_diff <= detection_angle_limit:
            # 가까운 지점에서 경로 중간 지점까지의 선분을 따라 장애물 확인
            for point in self.object_points:
                obj_x = point[0]
                obj_y = point[1]

                # 장애물이 차량과 로컬 경로 점을 잇는 선분에 근접한지 확인 (직선 거리)
                distance_to_line_closest = self.point_to_line_distance(ego_x, ego_y, closest_point.x, closest_point.y, obj_x, obj_y)
                distance_to_line_middle = self.point_to_line_distance(ego_x, ego_y, middle_point.x, middle_point.y, obj_x, obj_y)

                # 장애물이 부채꼴 영역 내에 있는지 확인
                if (distance_to_line_closest < 2 or distance_to_line_middle < 2) and \
                sqrt(pow(obj_x - ego_x, 2) + pow(obj_y - ego_y, 2)) < radius:
                    rospy.loginfo(f"Obstacle detected near path. Distance to path: {distance_to_line_closest}")
                    return True

        #rospy.loginfo("No obstacle detected near target point.")
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
        #self.object_points = current_object_points

        #rospy.loginfo(f"Received and stored {len(self.object_points)} static obstacle points.")
    
    def mission_callback(self, msg):
        try:
            self.mission_info = msg
            if self.mission_info.mission_num == 10:  # 예시로 미션 번호가 11이면 종료로 처리
                self.is_mission_end = True
            else:
                self.is_mission_end = False
        except ValueError as e:
            rospy.logerr(f"Invalid driving mode value: {self.mission_info.mission_num}")

    def filter_dynamic_objects(self, current_points):
        """
        8프레임 중 3프레임에서 장애물이 발견되면 정적 장애물로 간주.
        """
        static_objects = []
        threshold_distance = 0.6  # 동적/정적 객체를 구분할 거리 기준
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

        # 차량의 현재 위치
        ego_x = vehicle_status.pose.pose.position.x
        ego_y = vehicle_status.pose.pose.position.y

        # 차량 앞 1미터 지점을 계산 (헤딩 방향으로 1m 앞)
        start_x = ego_x + cos(heading) * 3.0  # 헤딩 방향으로 1미터 앞
        start_y = ego_y + sin(heading) * 3.0

        # 로컬 경로에서 가장 가까운 점의 y 오차 계산
        min_dist = float('inf')
        closest_pose = None
        for pose in self.local_path.poses:
            path_x = pose.pose.position.x
            path_y = pose.pose.position.y

            # 현재 위치에서 경로 점까지의 거리를 계산
            dist = sqrt(pow(path_x - start_x, 2) + pow(path_y - start_y, 2))

            if dist < min_dist:
                min_dist = dist
                closest_pose = pose

        if closest_pose is None:
            rospy.logwarn("No closest point found in local path.")
            return out_path

        # y방향 오차 계산 (차량의 헤딩 방향과 경로점의 상대적 위치)
        closest_x = closest_pose.pose.position.x
        closest_y = closest_pose.pose.position.y

        rel_x = cos(heading) * (closest_x - start_x) + sin(heading) * (closest_y - start_y)
        rel_y = -sin(heading) * (closest_x - start_x) + cos(heading) * (closest_y - start_y)
        # 기존 경로 생성 로직 (y 오차가 10m 이하일 경우 계속)
        offset = 0.3  # 오차를 얼마나 반영할지 결정하는 계수 (필요시 조정 가능)
        offset_value_x = rel_x * offset
        offset_value_y = rel_y * offset  # y 오차에 비례하여 보정 값 설정
        
        # y 오차가 9.8m 이상일 경우 경로 생성 중단, end_pose까지 직선 경로 설정
        if abs(rel_y) > 9.8:
            rospy.logwarn("Y error exceeds 10 meters! Stopping path generation and using end_pose only.")

            # end_pose를 헤딩 방향으로 설정
            end_x = ego_x + (min(max(1.8 * self.lfd, 18.0), 40.0)) * cos(heading)
            end_y = ego_y + (min(max(1.8 * self.lfd, 18.0), 40.0)) * sin(heading)

            # 직선 경로 생성
            lattice_path = Path()
            lattice_path.header.frame_id = '/map'
            x_interval = 0.25
            x_num = int(end_x / x_interval)

            for i in range(x_num):
                x = i * x_interval
                y = ego_y + (x / (end_x - ego_x)) * (end_y - ego_y)

                pose = PoseStamped()
                pose.pose.position.x = ego_x + x * cos(heading)
                pose.pose.position.y = ego_y + x * sin(heading)
                pose.pose.position.z = 0
                pose.pose.orientation.w = 1.0

                lattice_path.poses.append(pose)

            out_path.append(lattice_path)

            return out_path  # 즉시 직선 경로 반환

        

        if abs(rel_y) >= self.hderr_threshold:
            heading_end_x = ego_x + (min(max(1.7 * self.lfd, 17.0), 40.0)) * cos(heading)
            heading_end_y = ego_y + (min(max(1.7 * self.lfd, 17.0), 40.0)) * sin(heading)

            if len(self.local_path.poses) > 0:
                local_lfd_index = min(len(self.local_path.poses) - 1, int(min(max(2.0 * self.lfd, 20.0), 50.0)))
                local_end_x = self.local_path.poses[local_lfd_index].pose.position.x
                local_end_y = self.local_path.poses[local_lfd_index].pose.position.y
            else:
                rospy.logwarn("Local path is empty.")
                return out_path

            end_x = (heading_end_x + local_end_x) / 2.0 +offset_value_x
            end_y = (heading_end_y + local_end_y) / 2.0 + offset_value_y
        else:
            if len(self.local_path.poses) > 0:
                heading_end_x = ego_x + (min(max(1.7 * self.lfd, 17.0), 40.0)) * cos(heading)
                heading_end_y = ego_y + (min(max(1.7 * self.lfd, 17.0), 40.0)) * sin(heading)
                end_x = heading_end_x + offset_value_x
                end_y = heading_end_y + offset_value_y
            else: 
                heading_end_x = ego_x + 5 * cos(heading)
                heading_end_y = ego_y + 5 * sin(heading)
                rospy.logwarn("Local path is empty.")
                return out_path

        rel_end_x = cos(heading) * (end_x - ego_x) + sin(heading) * (end_y - ego_y)
        rel_end_y = -sin(heading) * (end_x - ego_x) + cos(heading) * (end_y - ego_y)

        end_point_msg = PointStamped()
        end_point_msg.header.frame_id = "map"
        end_point_msg.point.x = rel_end_x
        end_point_msg.point.y = rel_end_y
        end_point_msg.point.z = 0
        self.end_pose_point_pub.publish(end_point_msg)

        # 변환 행렬을 사용하여 좌표 변환 (차량 좌표계에서 전역 좌표계로 변환)
        translation = [ego_x, ego_y]
        theta = atan2(end_y - ego_y, end_x - ego_x)

        trans_matrix = np.array([[cos(theta), -sin(theta), translation[0]],
                                [sin(theta), cos(theta), translation[1]],
                                [0, 0, 1]])

        det_trans_matrix = np.array([[trans_matrix[0][0], trans_matrix[1][0], -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])],
                                    [trans_matrix[0][1], trans_matrix[1][1], -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                                    [0, 0, 1]])

        world_end_point = np.array([[end_x], [end_y], [1]])
        local_end_point = det_trans_matrix.dot(world_end_point)

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
            ps = local_end_point[1][0]
            pf = end_point[1]
            x_num = xf / x_interval

            for i in range(xs, int(x_num)):
                x.append(i * x_interval)

            a = [0.0, 0.0, 0.0, 0.0]
            a[0] = ps
            a[1] = 0
            a[2] = 6.4 * (pf - ps) / (xf * xf)
            a[3] = -5.9 * (pf - ps) / (xf * xf * xf)

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
    
