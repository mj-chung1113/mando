#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float32
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.clusterpoints_pub = rospy.Publisher("/cluster_points", PointCloud2, queue_size=10)
        self.warn_pub = rospy.Publisher("/warn_signal", Float32, queue_size=10)
        self.pc_np = None
        self.min_distance = float('inf')  # 초기 최소 거리를 무한대로 설정
        self.warn = 0.0  # 경고 신호 초기화

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return

        # 2. Intensity 값을 0~1로 스케일링
        self.pc_np[:, 3] = self.pc_np[:, 3] / 510  # intensity가 4번째 컬럼에 있음

        # Z 좌표를 0으로 조정 
        self.pc_np[:, 2] = self.pc_np[:, 2] * 0.2
    
        # 3. 거리별로 클러스터링 수행 (xyz + intensity 기반)
        cluster_points = []
        cluster_intensity_averages = []  # 디버깅용
        self.min_distance = float('inf')  # 각 콜백마다 최소 거리 초기화
        closest_cluster_intensity = None  # 가장 가까운 클러스터의 intensity를 저장할 변수

        for dist_range, params in self.get_dbscan_params_by_distance().items():
            # 해당 거리 범위의 포인트 필터링
            mask = (self.pc_np[:, 4] >= dist_range[0]) & (self.pc_np[:, 4] < dist_range[1])
            pc_xyz_intensity = self.pc_np[mask, :4]  # x, y, z, 스케일링된 intensity 포함

            if len(pc_xyz_intensity) == 0:
                continue

            # DBSCAN 적용 (xyz와 스케일링된 intensity를 함께 사용)
            dbscan = DBSCAN(eps=params['eps'], min_samples=params['min_samples'])
            db = dbscan.fit_predict(pc_xyz_intensity)
            n_cluster = np.max(db) + 1

            for c in range(n_cluster):
                # 클러스터의 평균 포인트와 intensity 평균을 계산
                c_points = pc_xyz_intensity[db == c, :]
                c_xyz_mean = np.mean(c_points[:, :3], axis=0)  # xyz 좌표 평균
                c_intensity_mean = np.mean(c_points[:, 3])  # intensity 평균 (스케일링된 값)

                cluster_points.append([c_xyz_mean[0], c_xyz_mean[1], c_xyz_mean[2], c_intensity_mean])
                cluster_intensity_averages.append(c_intensity_mean)  # intensity 평균 저장

                # 가장 가까운 클러스터의 거리를 계산
                cluster_distance = np.sqrt(c_xyz_mean[0]**2 + c_xyz_mean[1]**2)  # x, y 좌표 기반으로 거리 계산
                if cluster_distance < self.min_distance:
                    self.min_distance = cluster_distance
                    closest_cluster_intensity = c_intensity_mean  # 가장 가까운 클러스터의 intensity 저장

        # 가장 가까운 클러스터의 스케일링된 intensity 출력
        if closest_cluster_intensity is not None:
            rospy.loginfo(f"Closest Cluster Intensity (scaled): {closest_cluster_intensity}")

        # 차량 전방 20미터 이내 클러스터의 경고 신호 계산 및 퍼블리시
        self.calculate_warn_signal(cluster_points)

        # 최소 거리 로그 출력
        rospy.loginfo(f"Minimum Distance to Clustered Obstacle: {self.min_distance} meters")

        # 각 클러스터의 대표 포인트 및 평균 intensity로만 이루어진 포인트 클라우드 퍼블리시
        self.publish_point_cloud(cluster_points, cluster_intensity_averages)

    def calculate_warn_signal(self, cluster_points):
        """
        차량 전방 21.33미터 이내, 차량 x축 기준으로 가까운 장애물일수록 warn 값을 높이고
        각도가 0도에 가까울수록 warn 값을 높인다. 거리가 3.33미터에 가까울수록 warn 값이 1에 가깝고,
        21.33미터에 가까울수록 warn 값을 낮춘다. 3.33미터보다 가까운 경우 warn 값은 1이다.
        """
        for point in cluster_points:
            # 클러스터 포인트에는 [x, y, z, intensity] 네 개의 값이 저장되어 있음
            x, y, z, intensity = point
            distance = np.sqrt(x**2 + y**2)

            # 빨간색에 해당하는 intensity 범위 설정 (예: 스케일링된 값이 0.6~0.8 사이)
            is_red_intensity = 0.168 <= intensity <= 0.188
            is_box_intensity = 0.091 <= intensity <= 0.12
            is_poll_intensity =  0.022 <= intensity <= 0.045 
            is_road_intensity = 0.045 < intensity <= 0.051
            is_baricade_intensity = 0.099 < intensity <= 0.11
            if 40 > x > 0 and -3.0 < y < 3.0 and -1.40 * 0.2 < z and distance < 40 or is_box_intensity or is_red_intensity:
                angle = abs(np.arctan2(y, x))  # 차량 전방과의 각도 (라디안)
                angle_scale = max(0, 1 - (angle / (np.pi / 3)))  # ±60도 기준으로 스케일링

                # 거리가 3.33 미터보다 가까운 경우 warn을 1로 설정
                if distance <= 11:
                    distance_scale = 1
                else:
                    # 거리는 3.33m에서 21.33m로 스케일링. 가까울수록 warn이 1, 멀수록 0
                    distance_scale = min(1.00, max(0.001, 1 - ((distance - 11) / (40 - 11))))

                #  각도/거리 조건 만족 시 경고 신호
                if  (angle_scale * distance_scale) > self.warn :
                    self.warn = max(self.warn, angle_scale * distance_scale)
                    if is_box_intensity or is_red_intensity or is_baricade_intensity: 
                        self.warn = min(1.0,self.warn*1.2)

                    elif is_poll_intensity:
                        self.warn = self.warn*0.5
                    elif is_road_intensity:
                        self.warm = self.warn*0.1

        # 최대 warn 값 퍼블리시
        self.warn_pub.publish(Float32(self.warn))
        rospy.loginfo(f"Warn signal: {self.warn}")
        self.warn = min(1, max(0.0001, 0.83 * self.warn))

    def remove_noise_by_intensity(self, points, noise_threshold=0.1):
        """
        intensity 값이 noise_threshold 이하인 포인트는 노이즈로 간주하여 제거.
        """
        intensity_values = points[:, 3]  # intensity 값은 4번째 컬럼에 있음
        mask = intensity_values > noise_threshold  # noise_threshold보다 큰 값만 남김
        rospy.loginfo(f"Noise Threshold: {noise_threshold}, Points Remaining: {np.sum(mask)}")
        return points[mask]

    def get_dbscan_params_by_distance(self):
        """
        거리 범위에 따라 DBSCAN의 eps와 min_samples 파라미터를 설정합니다.
        """
        return {
            (2, 5): {'eps': 0.1, 'min_samples': 20},
            (5, 9): {'eps': 0.19, 'min_samples': 11},  # intensity 포함 -> eps 값 조정
            (9, 19): {'eps': 0.34, 'min_samples': 9},
            (19, 29): {'eps': 0.43, 'min_samples': 7},
            (29, 45): {'eps': 0.47, 'min_samples': 5},
        }

    def publish_point_cloud(self, points, intensity_averages):
        """
        클러스터 대표 포인트들과 해당 클러스터의 평균 intensity를 포함한 PointCloud2 메시지를 생성하여 퍼블리시.
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),  # intensity 필드 추가
        ]

        # PointCloud2 메시지 생성
        points_with_intensity = [[p[0], p[1], p[2], intensity] for p, intensity in zip(points, intensity_averages)]
        pc2_msg = pc2.create_cloud(header, fields, points_with_intensity)

        # PointCloud2 메시지 발행s
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2)
            angle = np.arctan2(point[1], point[0])

            # point[0] = x / point[1] = y / point[2] = z / point[3] = intensity
            if -0.5 < point[0] < 45 and -10 < point[1] < 10 and (0.5 < dist < 45) and (-1.48 < point[2] < 0.2):
                point_list.append((point[0], point[1], 0, point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin()
