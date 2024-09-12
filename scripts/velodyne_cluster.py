#!/usr/bin/env python3
#-*- coding:utf-8 -*-

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

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return
        
        # 모든 포인트의 Z 좌표를 -0.5로 설정
        self.pc_np[:, 2] = -1.3
        
        # 거리별로 클러스터링 수행
        cluster_points = []
        for dist_range, params in self.get_dbscan_params_by_distance().items():
            # 해당 거리 범위의 포인트 필터링
            mask = (self.pc_np[:, 4] >= dist_range[0]) & (self.pc_np[:, 4] < dist_range[1])
            pc_xyz = self.pc_np[mask, :3]  # x, y, z 좌표 모두 포함

            if len(pc_xyz) == 0:
                continue

            # DBSCAN 적용
            dbscan = DBSCAN(eps=params['eps'], min_samples=params['min_samples'])
            db = dbscan.fit_predict(pc_xyz)
            n_cluster = np.max(db) + 1

            for c in range(n_cluster):
                c_tmp = np.mean(pc_xyz[db == c, :], axis=0)
                cluster_points.append([c_tmp[0], c_tmp[1], c_tmp[2]])  # Z 좌표 포함

        self.publish_point_cloud(cluster_points)

    def remove_floor_plane(self, points):
        """
        RANSAC을 사용하여 바닥 평면을 제거합니다.
        """
        X = points[:, :3]  # x, y, z 좌표
        z_values = X[:, 2]

        # 바닥 평면을 고려할 Z-값 범위
        z_min = -1.36
        z_max = -1.28

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
                (0, 10): {'eps': 0.1, 'min_samples': 20},
                (10, 30): {'eps': 0.3, 'min_samples': 10},
                (30, 50): {'eps': 0.45, 'min_samples': 7},
                
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

        # PointCloud2 메시지 생성
        pc2_msg = pc2.create_cloud(header, fields, points)

        # PointCloud2 메시지 발행
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            
            # point[0] = x / point[1] = y / point[2] = z
            if -1 < point[0] < 70 and -3 < point[1] < 3 and (dist < 70) and (-1.33 < point[2] < 0.2):
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))
            
        point_np = np.array(point_list, np.float32)
        return point_np

if __name__ == '__main__':
    scan_cluster = SCANCluster()
    rospy.spin()
