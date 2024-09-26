#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.clusterpoints_pub = rospy.Publisher("/cluster_points", PointCloud2, queue_size=10)
        self.pc_np = None
        self.min_distance = float('inf')  # 초기 최소 거리를 무한대로 설정

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return
        
        # 1. Intensity 값 기반 노이즈 제거 (기본값 0.1 이하 제거)
        self.pc_np = self.remove_noise_by_intensity(self.pc_np)

        # 2. 바닥 평면 제거 (필요하면 주석 해제)
        # self.pc_np = self.remove_floor_plane(self.pc_np)

        # Z 좌표를 0으로 조정
        self.pc_np[:, 2] = 0.0
        
        # 3. 거리별로 클러스터링 수행 (xyz + intensity 기반)
        cluster_points = []
        cluster_intensity_averages = []
        self.min_distance = float('inf')  # 각 콜백마다 최소 거리 초기화

        for dist_range, params in self.get_dbscan_params_by_distance().items():
            # 해당 거리 범위의 포인트 필터링
            mask = (self.pc_np[:, 4] >= dist_range[0]) & (self.pc_np[:, 4] < dist_range[1])
            pc_xyz_intensity = self.pc_np[mask, :4]  # x, y, z, intensity 포함

            if len(pc_xyz_intensity) == 0:
                continue

            # DBSCAN 적용 (xyz와 intensity를 함께 사용)
            dbscan = DBSCAN(eps=params['eps'], min_samples=params['min_samples'])
            db = dbscan.fit_predict(pc_xyz_intensity)
            n_cluster = np.max(db) + 1

            for c in range(n_cluster):
                # 클러스터의 평균 포인트와 intensity 평균을 계산
                c_points = pc_xyz_intensity[db == c, :]
                c_xyz_mean = np.mean(c_points[:, :3], axis=0)  # xyz 좌표 평균
                c_intensity_mean = np.mean(c_points[:, 3])  # intensity 평균

                cluster_points.append([c_xyz_mean[0], c_xyz_mean[1], c_xyz_mean[2]])  # xyz 좌표 포함
                cluster_intensity_averages.append(c_intensity_mean)  # intensity 평균 저장

                # 클러스터의 평균 포인트로부터 최소 거리 계산
                distance_to_cluster = np.linalg.norm(c_xyz_mean[:2])  # x, y 좌표를 기준으로 거리 계산
                if distance_to_cluster < self.min_distance:
                    self.min_distance = distance_to_cluster

            # 평균 intensity 로그 출력
            rospy.loginfo(f"Distance Range: {dist_range}, Cluster Intensity Average: {cluster_intensity_averages}")

        # 최소 거리 로그 출력
        rospy.loginfo(f"Minimum Distance to Clustered Obstacle: {self.min_distance} meters")

        # 각 클러스터의 대표 포인트 및 평균 intensity로만 이루어진 포인트 클라우드 퍼블리시
        self.publish_point_cloud(cluster_points, cluster_intensity_averages)

    def remove_noise_by_intensity(self, points, noise_threshold=0.1):
        """
        intensity 값이 noise_threshold 이하인 포인트는 노이즈로 간주하여 제거.
        """
        intensity_values = points[:, 3]  # intensity 값은 4번째 컬럼에 있음
        mask = intensity_values > noise_threshold  # noise_threshold보다 큰 값만 남김
        rospy.loginfo(f"Noise Threshold: {noise_threshold}, Points Remaining: {np.sum(mask)}")
        return points[mask]

    def remove_floor_plane(self, points):
        """
        RANSAC을 사용하여 바닥 평면을 제거합니다.
        """
        X = points[:, :3]  # x, y, z 좌표
        z_values = X[:, 2]

        # 바닥 평면을 고려할 Z-값 범위
        z_min = -1.49
        z_max = -1.4

        # Z-값이 범위 내에 있는 포인트만 필터링
        floor_points = X[(z_values >= z_min) & (z_values <= z_max)]

        # RANSAC 적용
        ransac = RANSACRegressor(residual_threshold=0.05)
        ransac.fit(floor_points[:, :2], floor_points[:, 2])  # x, y를 독립 변수, z를 종속 변수로 사용
        inlier_mask = ransac.inlier_mask_

        # 바닥 평면에 해당하는 포인트를 제외하고 나머지 포인트 반환
        filtered_points = points[~(np.isin(np.arange(len(points)), np.where((z_values >= z_min) & (z_values <= z_max))[0]))]
        return filtered_points

    def get_dbscan_params_by_distance(self):
        """
        거리 범위에 따라 DBSCAN의 eps와 min_samples 파라미터를 설정합니다.
        """
        return {
            (2, 5): {'eps': 0.1, 'min_samples': 30},
            (5, 10): {'eps': 0.18, 'min_samples': 15},  # intensity 포함 -> eps 값 조정
            (10, 20): {'eps': 0.33, 'min_samples': 13},
            (20, 30): {'eps': 0.45, 'min_samples': 10},
            (30, 40): {'eps': 0.47, 'min_samples': 7},
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

        # PointCloud2 메시지 발행
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2)
            angle = np.arctan2(point[1], point[0])

            # point[0] = x / point[1] = y / point[2] = z / point[3] = intensity
            if -2 < point[0] < 45 and -5 < point[1] < 5 and (1.5< dist < 45) and (-1.4 < point[2] < 0):
                point_list.append((point[0], point[1], 0, point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin()
