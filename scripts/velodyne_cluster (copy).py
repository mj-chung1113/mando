#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from morai_msgs.msg import EgoVehicleStatus
from sklearn.linear_model import RANSACRegressor

class SCANCluster:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.clusterpoints_pub = rospy.Publisher("/cluster_points", PointCloud2, queue_size=10)
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)

        self.pc_np = None
        self.prev_clusters = []  # 이전 프레임 클러스터들의 중심점 정보를 저장
        self.frame_count = 0  # 프레임 카운터
        self.max_frames = 10  # 추적할 최대 프레임 수
        self.velocity = 0  # 차량 속도 초기화
        
        self.min_distance = float('inf')  # 초기 최소 거리를 무한대로 설정
    
    def status_callback(self, msg):
        """
        차량의 속도를 EgoVehicleStatus에서 업데이트
        """
        self.velocity = np.sqrt(msg.velocity.x**2 + msg.velocity.y**2)
        rospy.loginfo(f"Vehicle velocity: {self.velocity} m/s")

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return
        
        # 1. Intensity 값 기반 노이즈 제거
        self.pc_np = self.remove_noise_by_intensity(self.pc_np)
        
        # Z 좌표를 0으로 조정
        self.pc_np[:, 2] = 0.0
        
        # 3. 클러스터링 수행
        cluster_points = []
        self.min_distance = float('inf')  # 각 콜백마다 최소 거리 초기화

        for dist_range, params in self.get_dbscan_params_by_distance().items():
            # 거리 기준 필터링 적용 (pc_np의 5번째 컬럼이 거리)
            mask = (self.pc_np[:, 4] >= dist_range[0]) & (self.pc_np[:, 4] < dist_range[1])
            pc_xyz_intensity = self.pc_np[mask, :4]  # 거리로 필터링 후 클러스터링

            if len(pc_xyz_intensity) == 0:
                continue

            dbscan = DBSCAN(eps=params['eps'], min_samples=params['min_samples'])
            db = dbscan.fit_predict(pc_xyz_intensity)
            n_cluster = np.max(db) + 1

            for c in range(n_cluster):
                c_points = pc_xyz_intensity[db == c, :]
                c_xyz_mean = np.mean(c_points[:, :3], axis=0)  # 클러스터 중심점
                cluster_points.append(c_xyz_mean)  # 중심점 저장

        # 현재 프레임 클러스터 업데이트 및 비교
        static_clusters = self.filter_dynamic_objects(cluster_points)

        # 정적 장애물 퍼블리시
        self.publish_point_cloud(static_clusters)

    def filter_dynamic_objects(self, cluster_points):
        """
        이전 프레임과 현재 프레임의 클러스터 포인트를 비교하여 동적 객체를 제거.
        일정 프레임 동안 변화가 없는 정적 객체만 남김.
        """
        # 차량 속도를 고려한 변화량 임계값
        threshold_distance = self.velocity * 0.15  # 차량 속도에 비례하여 변화량 기준 설정
        
        if len(self.prev_clusters) == 0:
            # 첫 번째 프레임에서는 비교할 이전 클러스터가 없으므로, 모든 클러스터 저장
            self.prev_clusters = [{'center': point, 'frame_count': 0} for point in cluster_points]
            return cluster_points
        
        static_clusters = []
        
        for prev_cluster in self.prev_clusters:
            prev_point = np.array(prev_cluster['center'])  # 이전 프레임 클러스터 중심점

            for current_point in cluster_points:
                current_point_np = np.array(current_point)

                # 클러스터 간 거리 계산 (x, y 좌표 기준으로 비교)
                distance = np.linalg.norm(prev_point[:2] - current_point_np[:2])
                
                if distance > threshold_distance:
                    # 변화량이 작으면 정적 객체로 간주
                    prev_cluster['frame_count'] += 1
                    if prev_cluster['frame_count'] >= self.max_frames:
                        static_clusters.append(current_point)
                else:
                    # 변화량이 크면 동적 객체로 간주하여 무시
                    prev_cluster['frame_count'] = 0

        # 현재 클러스터 중심점을 다음 프레임 비교를 위해 저장
        self.prev_clusters = [{'center': point, 'frame_count': 0} for point in cluster_points]
        
        return static_clusters

    def remove_noise_by_intensity(self, points, noise_threshold=0.1):
        intensity_values = points[:, 3]
        mask = intensity_values > noise_threshold
        return points[mask]

    def get_dbscan_params_by_distance(self):
        return {
            (2, 5): {'eps': 0.1, 'min_samples': 30},
            (5, 10): {'eps': 0.18, 'min_samples': 15},
            (10, 20): {'eps': 0.33, 'min_samples': 13},
            (20, 30): {'eps': 0.45, 'min_samples': 10},
            (30, 40): {'eps': 0.47, 'min_samples': 7},
        }

    def publish_point_cloud(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, points)
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        """
        PointCloud2 메시지를 받아 (x, y, z, intensity, dist, angle) 배열로 변환
        """
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2)  # 거리 계산
            angle = np.arctan2(point[1], point[0])     # 각도 계산
            if -2 <point[0]<40 and -3<point[1]<3 and 1.5 < dist < 45 and (-1.4 < point[2] < 0):  # 필터링 조건 적용
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin()
