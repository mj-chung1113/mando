#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage, Image
from mando.msg import mission, obstacle
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16, Bool, Float32

import cv2
import numpy as np
from ultralytics import YOLO
import torch
from collections import deque
from statistics import mode

class YoloNode:
    def __init__(self):
        rospy.init_node('yolo_node', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.mode_sub = rospy.Subscriber("/mission", mission, self.mission_callback)
        
        self.obstacle_pub = rospy.Publisher("/obstacle_info", obstacle, queue_size=5)
        self.traffic_color_pub = rospy.Publisher("/traffic_light_color", Int16, queue_size=10)
        self.stop_pub = rospy.Publisher("/stop_signal", Bool, queue_size=1)
        self.roi_image_pub = rospy.Publisher("/roi_visualization", Image, queue_size=5)
        self.speed_adjust_pub = rospy.Publisher("/speed_adjust_signal", Float32, queue_size=1)

        self.bridge = CvBridge()

        # GPU 사용 설정
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.model = YOLO('yolov8n.pt').to(self.device)  # YOLO 모델 로드
        
        self.confidence_threshold = 0.2
        self.mission_info = mission()
        self.obstacle = obstacle()
        
        self.timer = rospy.Timer(period=rospy.Duration(0.1), callback=self.timer_callback)
        self.latest_frame = None
        self.previous_boxes = {}
        self.recent_traffic_colors = deque(maxlen=5)

        rospy.loginfo("YOLO node has been started.")

    # CUDA 사용을 위한 이미지 전처리
    def preprocess_image(self, frame):
        if frame is None or frame.size == 0:
            rospy.logerr("Preprocess image received an empty frame.")
            return None

        resized_frame = cv2.resize(frame, (640, 640))
        rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        rgb_frame = np.transpose(rgb_frame, (2, 0, 1))
        tensor_frame = torch.from_numpy(rgb_frame).float().unsqueeze(0)
        tensor_frame /= 255.0
        
        return tensor_frame.to(self.device)

    # ROI 설정
    def set_roi_by_mission(self, mission_num, frame_width, frame_height):
        if mission_num == 2 or mission_num == 6: 
            roi_x1 = int(frame_width * 0.44)
            roi_x2 = int(frame_width * 0.56)
            roi_y1 = int(frame_height * 0.30)
            roi_y2 = int(frame_height * 0.50)
        else:
            roi_x1 = int(frame_width * 0.44)
            roi_x2 = int(frame_width * 0.56)
            roi_y1 = int(frame_height * 0.30)
            roi_y2 = int(frame_height * 0.50)

        if self.latest_frame is not None:
            self.visualize_and_publish_roi(self.latest_frame, roi_x1, roi_y1, roi_x2, roi_y2)

        return roi_x1, roi_y1, roi_x2, roi_y2

    # IOU 계산 함수 추가
    def calculate_iou(self, x1, y1, x2, y2, roi_x1, roi_y1, roi_x2, roi_y2):
        overlap_x1 = max(x1, roi_x1)
        overlap_y1 = max(y1, roi_y1)
        overlap_x2 = min(x2, roi_x2)
        overlap_y2 = min(y2, roi_y2)

        overlap_width = max(0, overlap_x2 - overlap_x1)
        overlap_height = max(0, overlap_y2 - overlap_y1)
        overlap_area = overlap_width * overlap_height

        box_area = (x2 - x1) * (y2 - y1)
        roi_area = (roi_x2 - roi_x1) * (roi_y2 - roi_y1)

        union_area = box_area + roi_area - overlap_area
        if union_area == 0:
            return 0.0

        iou = overlap_area / union_area
        return iou

    # 신뢰도 낮은 것과 class_number가 0, 1, 2, 3가 아닌 것을 버리기
    def filter_results_by_confidence(self, results):
        allowed_classes = {0, 1, 2, 3, 888}
        filtered_results = []
        for result in results:
            filtered_boxes = [box for box in result.boxes if box.conf > self.confidence_threshold and int(box.cls.item()) in allowed_classes]
            filtered_results.append(filtered_boxes)
        return filtered_results

    # 보행자 충돌 확률 계산 (두 가지 방법의 평균)
    def calculate_collision_probability(self, frame):
        original_height, original_width = frame.shape[:2]

        tensor_frame = self.preprocess_image(frame)
        results = self.model(tensor_frame)
        filtered_results = self.filter_results_by_confidence(results)

        pedestrian_detected = False
        total_collision_probability = 0.0
        num_methods = 2  # 두 가지 방법 사용

        # ROI 설정 및 가중치
        basic_roi_x1, basic_roi_x2, basic_roi_y1, basic_roi_y2 = int(original_width * 0.10), int(original_width * 0.90), int(original_height * 0.35), int(original_height * 0.85)
        road_roi_x1, road_roi_x2, road_roi_y1, road_roi_y2 = int(original_width * 0.30), int(original_width * 0.70), int(original_height * 0.40), int(original_height * 0.85)
        line_roi_x1, line_roi_x2, line_roi_y1, line_roi_y2 = int(original_width * 0.40), int(original_width * 0.60), int(original_height * 0.50), int(original_height * 0.85)

        basic_roi_weight = 1
        road_roi_weight = 3
        line_roi_weight = 5

        for result in filtered_results:
            for box in result:
                class_id = int(box.cls.item())
                if class_id != 0:
                    continue

                pedestrian_detected = True

                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                x1 = int(x1 * original_width / 640)
                x2 = int(x2 * original_width / 640)
                y1 = int(y1 * original_height / 640)
                y2 = int(y2 * original_height / 640)

                # 방법 1: 중심점 거리 기반 계산
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                distance_ratio = self.calculate_distance_ratio(center_x, center_y, original_width // 2, original_height // 2, original_width, original_height)
                collision_probability_distance = (1 - distance_ratio) * 100.0

                # 방법 2: ROI 포함 비율 기반 계산 (각 ROI 가중치 적용)
                basic_overlap_area = self.calculate_overlap_area(x1, y1, x2, y2, basic_roi_x1, basic_roi_y1, basic_roi_x2, basic_roi_y2)
                road_overlap_area = self.calculate_overlap_area(x1, y1, x2, y2, road_roi_x1, road_roi_y1, road_roi_x2, road_roi_y2)
                line_overlap_area = self.calculate_overlap_area(x1, y1, x2, y2, line_roi_x1, line_roi_y1, line_roi_x2, line_roi_y2)

                box_area = (x2 - x1) * (y2 - y1)
                basic_ratio = (basic_overlap_area / box_area) * basic_roi_weight if box_area > 0 else 0
                road_ratio = (road_overlap_area / box_area) * road_roi_weight if box_area > 0 else 0
                line_ratio = (line_overlap_area / box_area) * line_roi_weight if box_area > 0 else 0
                collision_probability_roi = (basic_ratio + road_ratio + line_ratio) * 100.0 / (basic_roi_weight + road_roi_weight + line_roi_weight)

                # 충돌 확률 계산 (각 방법의 평균 사용)
                collision_probability = (collision_probability_distance + collision_probability_roi) / num_methods

                # 각 방법의 충돌 확률을 로그로 출력
                # rospy.loginfo(f"방법 1 (중심점 거리 기반) 충돌 가능성: {collision_probability_distance:.2f}%")
                # rospy.loginfo(f"방법 2 (ROI 기반) 충돌 가능성: {collision_probability_roi:.2f}%")
                # rospy.loginfo(f"최종 충돌 가능성: {collision_probability:.2f}%")

                # 최종 평균 충돌 확률 갱신
                total_collision_probability = (collision_probability_distance + collision_probability_roi) / num_methods

                # 객체 시각화
                label_text = f"Person, Conf: {box.conf.item():.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    
        # 충돌 확률에 따른 행동 결정
        if pedestrian_detected:
            
            self.speed_adjust_pub.publish(0.01*total_collision_probability)
            rospy.loginfo(f"보행자 검출, 충돌 확률 : {total_collision_probability}")
        else:
            # 보행자가 없을 경우 신호 해제 및 속도 원상 복귀
            
            self.speed_adjust_pub.publish(1.0)
            rospy.loginfo("보행자 검출되지 않음.")


    # 보행자 이미지 크기에 대한 가중치 계산
    def calculate_size_weight(self, box_area, frame_area, min_weight=0.2, max_weight=1.0):
        size_ratio = box_area / frame_area
        weight = size_ratio * max_weight
        # 최소 가중치와 최대 가중치 사이로 제한
        weight = max(min_weight, min(weight, max_weight))
        return weight

    # 중심점 거리 비율 계산
    def calculate_distance_ratio(self, x, y, center_x, center_y, width, height):
        distance = ((x - center_x) ** 2 + (y - center_y) ** 2) ** 0.5
        max_distance = ((width / 2) ** 2 + (height / 2) ** 2) ** 0.5
        return distance / max_distance

    # 겹치는 영역 계산
    def calculate_overlap_area(self, x1, y1, x2, y2, roi_x1, roi_y1, roi_x2, roi_y2):
        overlap_x1 = max(x1, roi_x1)
        overlap_y1 = max(y1, roi_y1)
        overlap_x2 = min(x2, roi_x2)
        overlap_y2 = min(y2, roi_y2)

        overlap_width = max(0, overlap_x2 - overlap_x1)
        overlap_height = max(0, overlap_y2 - overlap_y1)

        return overlap_width * overlap_height

    def visualize_and_publish_roi(self, frame, roi_x1, roi_y1, roi_x2, roi_y2):
        visualized_frame = frame.copy()
        cv2.rectangle(visualized_frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (20, 20, 20), 2)
        try:
            roi_image_msg = self.bridge.cv2_to_imgmsg(visualized_frame, "bgr8")
            self.roi_image_pub.publish(roi_image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert ROI visualized image: {e}")

    def image_callback(self, data):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            if frame is not None:
                self.latest_frame = frame
            else:
                rospy.logwarn("Received an empty frame. Skipping frame update.")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert image: {e}")
            self.latest_frame = None

    def mission_callback(self, msg):
        self.mission_info = msg
     # 신호등 색상 결정
    def decide_traffic_light_color(self):
        if self.recent_traffic_colors:
            return mode(self.recent_traffic_colors)
        else:
            return 0

    def detect_traffic_light_color(self, roi):
        # HSV 색 공간으로 변환
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 색 범위 설정 (예: 빨간색, 노란색, 초록색)
        red_lower = np.array([130, 120, 60])
        red_upper = np.array([179, 255, 255])   
        yellow_lower = np.array([10, 120, 175])
        yellow_upper = np.array([45, 255, 255])
        green_lower = np.array([40, 50, 145])
        green_upper = np.array([75, 255, 255])

        # 마스크 생성
        red_mask = cv2.inRange(hsv_roi, red_lower, red_upper)
        yellow_mask = cv2.inRange(hsv_roi, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv_roi, green_lower, green_upper)

        # 각 색깔의 픽셀 수 계산
        red_pixels = cv2.countNonZero(red_mask)
        yellow_pixels = cv2.countNonZero(yellow_mask)
        green_pixels = cv2.countNonZero(green_mask)

        # 가장 많은 픽셀 수를 가진 색깔을 감지
        if red_pixels > yellow_pixels and red_pixels > green_pixels:
            self.traffic_color = 1 #"red"
        elif yellow_pixels > red_pixels and yellow_pixels > green_pixels:
            self.traffic_color = 2 #"yellow"
        elif green_pixels > red_pixels and green_pixels > yellow_pixels:
            self.traffic_color = 3 #"green"
        else:
            self.traffic_color = 0 #"unknown"
        
        # 최근 색상 결과 업데이트
        self.recent_traffic_colors.append(self.traffic_color)
        rospy.loginfo(f"Current detected color: {self.traffic_color}")

        # 최종 색상 결정 및 퍼블리시
        self.final_color = self.decide_traffic_light_color()
        self.traffic_color_pub.publish(self.final_color)
        rospy.loginfo(f"Final traffic light color: {self.final_color}")

    
    def timer_callback(self, event):
        if self.latest_frame is not None:
            frame = self.latest_frame.copy()
            traffic = self.latest_frame.copy()
            # ROI 시각화
            basic_roi_x1, basic_roi_x2, basic_roi_y1, basic_roi_y2 = int(frame.shape[1] * 0.10), int(frame.shape[1] * 0.90), int(frame.shape[0] * 0.35), int(frame.shape[0] * 0.85)
            road_roi_x1, road_roi_x2, road_roi_y1, road_roi_y2 = int(frame.shape[1] * 0.30), int(frame.shape[1] * 0.70), int(frame.shape[0] * 0.40), int(frame.shape[0] * 0.85)
            line_roi_x1, line_roi_x2, line_roi_y1, line_roi_y2 = int(frame.shape[1] * 0.40), int(frame.shape[1] * 0.60), int(frame.shape[0] * 0.50), int(frame.shape[0] * 0.85)

            # ROI 영역을 시각적으로 표시
            cv2.rectangle(frame, (basic_roi_x1, basic_roi_y1), (basic_roi_x2, basic_roi_y2), (0, 255, 0), 2)
            cv2.rectangle(frame, (road_roi_x1, road_roi_y1), (road_roi_x2, road_roi_y2), (255, 0, 0), 2)
            cv2.rectangle(frame, (line_roi_x1, line_roi_y1), (line_roi_x2, line_roi_y2), (0, 255, 255), 2)

            # 충돌 확률 계산
            self.calculate_collision_probability(frame)
            
            roi_x1, roi_y1, roi_x2, roi_y2 = self.set_roi_by_mission(self.mission_info.mission_num, 
                                                                            self.latest_frame.shape[1], 
                                                                            self.latest_frame.shape[0])
            roi= traffic[roi_y1:roi_y2,roi_x1:roi_x2]
            # 신호등 검출 및 색상 분석
            self.detect_traffic_light_color(roi)
            # OpenCV 창에 이미지 표시
            cv2.imshow("YOLO & Collision Detection with ROIs", frame)
            cv2.waitKey(1)

def main():
    yolo_node = YoloNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLO node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
