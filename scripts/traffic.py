#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage, Image
from kurrier.msg import mission, obstacle  # 사용자 정의 메시지 임포트 
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int16
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
        self.obstacle_pub = rospy.Publisher("/obstacle_info", obstacle, queue_size=5)  # 새로운 퍼블리셔
        self.traffic_color_pub = rospy.Publisher("/traffic_light_color", Int16, queue_size=10 )
        
        self.bridge = CvBridge()
        self.roi_image_pub = rospy.Publisher("/roi_visualization", Image, queue_size=5)  # ROI 시각화 퍼블리셔 추가

        # GPU 사용 설정
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.path = 'yolov8n.pt'
        self.model = YOLO(self.path)
        self.model = self.model.to(self.device)
        # YOLO 모델을 GPU에 로드
        #self.general_model = YOLO('/home/yolov8n.pt').to(self.device)
        
        self.confidence_threshold = 0.4  # 신뢰도 임계값
        self.mission_info = mission()
        self.obstacle = obstacle()
        rospy.loginfo("YOLO node has been started.")
        
        self.timer = rospy.Timer(period=rospy.Duration(0.1), callback=self.timer_callback)
        self.latest_frame = None

        self.previous_boxes = {}
        self.previous_collision_probabilities = []

        #신호등 처리 
        self.traffic_color=0
        self.recent_traffic_colors = deque(maxlen=5)
        self.final_color = 0 

    #cuda 사용을 위한 이미지 전처리 
    def preprocess_image(self, frame):
        # 이미지가 비어있는지 확인
        if frame is None or frame.size == 0:
            rospy.logerr("Preprocess image received an empty frame.")
            return None

        # 이미지 크기를 (640, 640)으로 조정
        resized_frame = cv2.resize(frame, (640, 640))

        # 이미지를 RGB로 변환 (OpenCV는 기본적으로 BGR 형식이므로 변환 필요)
        rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

        # 이미지를 (H, W, C)에서 (C, H, W)로 변환
        rgb_frame = np.transpose(rgb_frame, (2, 0, 1))

        # PyTorch 텐서로 변환하고 배치 차원을 추가
        tensor_frame = torch.from_numpy(rgb_frame).float().unsqueeze(0)

        # 0-255 범위를 0-1로 정규화
        tensor_frame /= 255.0
        return tensor_frame.to(self.device)
        
    def set_roi_by_mission(self, mission_num, frame_width, frame_height):
 
        if mission_num == 1 or mission_num == 51: 
            # 차간간격 
            roi_x1 = int(frame_width * 0.36)
            roi_x2 = int(frame_width * 0.64)
            roi_y1 = int(frame_height * 0.4)
            roi_y2 = int(frame_height * 0.83)
        
        elif mission_num == 2 or mission_num == 6:
            # 정적장애물 및 동적(보행자 감지)
            roi_x1 = int(frame_width*0.05)
            roi_x2 = int(frame_width * 0.95)
            roi_y1 = int(frame_height * 0.35)
            roi_y2 = int(frame_height * 0.85)
            
            # 끼어들기
        elif mission_num == 5:
            roi_x1 = int(frame_width * 0.36)
            roi_x2 = int(frame_width * 0.64)
            roi_y1 = int(frame_height * 0.43)
            roi_y2 = int(frame_height * 0.75)

        elif mission_num == 71:
            #신호등 감지 미션, 상단 중앙 영역을 ROI로 설정
            
            roi_x1 = int(0.44 * frame_width)
            roi_x2 = int(0.56 * frame_width)
            roi_y1 = int(0.29 * frame_height)
            roi_y2 = int(0.43 * frame_height)
        
        else:
            # 기본 ROI 설정: 이미지 전체 영역
            roi_x1 = int(frame_width * 0.05)
            roi_x2 = int(frame_width * 0.95)
            roi_y1 = int(frame_height * 0.35)
            roi_y2 = int(frame_height * 0.85)
        
        if self.latest_frame is not None:
            self.visualize_and_publish_roi(self.latest_frame, roi_x1, roi_y1, roi_x2, roi_y2)
        return roi_x1, roi_y1, roi_x2, roi_y2
    
    
        #충돌 확률 계산 알고리즘 
    
    def calculate_distance_ratio(self, x, y, center_x, center_y, width, height):
        distance = ((x - center_x) ** 2 + (y - center_y) ** 2) ** 0.5
        max_distance = ((width / 2) ** 2 + (height / 2) ** 2) ** 0.5
        return distance / max_distance

    #충돌 확률 계산 알고리즘 
    def get_closest_point_to_center(self, x1, y1, x2, y2, center_x, center_y):
        points = [(x1, y1), (x1, y2), (x2, y1), (x2, y2)]
        closest_point = min(points, key=lambda p: (p[0] - center_x) ** 2 + (p[1] - center_y) ** 2)
        return closest_point
    
    #바운딩 박스 크기 계산 
    def calculate_box_area(self, x1, y1, x2, y2):
        return (x2 - x1) * (y2 - y1)

    def calculate_area_change_rate(self, object_id, current_area):
        if object_id in self.previous_boxes:
            previous_area = self.previous_boxes[object_id]
            area_change_rate = 5*(current_area - previous_area) / max(previous_area, 1)
        else:
            area_change_rate = 0.01

        # 현재 프레임의 바운딩 박스 크기를 저장
        self.previous_boxes[object_id] = current_area
        return area_change_rate

    #장애물 정보 관리 ,pub
    def publish_obstacle_info(self, classn, collision):
        
        self.obstacle.classnum = classn
        self.obstacle.collision_probability = collision
        self.obstacle_pub.publish(self.obstacle)
        rospy.loginfo(f"Published obstacle info: classnum={self.obstacle.classnum}, collision_probability={self.obstacle.collision_probability}")

    #바운딩박스 short term 예측 
    def predict_next_positions(self, results):
        predicted_boxes = []
        for boxes in results:
            for box in boxes:
                object_id = int(box.cls.item())
                current_box = box.xyxy[0].cpu().numpy()
                predicted_boxes.append((object_id, current_box, box.conf, box.cls))
        return predicted_boxes
    
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

    #충돌 확률 계산 알고리즘 
    def calculate_collision_probability(self, frame):
        try:
            # 이미지 전처리 및 GPU로 이동
            tensor_frame = self.preprocess_image(frame)
            results = self.model(tensor_frame)
            #results = self.general_model(tensor_frame)
            filtered_results = self.filter_results_by_confidence(results)

            if not filtered_results or all(len(boxes) == 0 for boxes in filtered_results):
                rospy.loginfo("No objects detected. Collision probability set to 0.")
                self.publish_obstacle_info(888, 0.0)
            else:
                predicted_boxes = self.predict_next_positions(filtered_results)
                classnum, highest_collision_probability = self.compute_highest_collision_probability(predicted_boxes, frame)
                self.publish_obstacle_info(classnum, highest_collision_probability)
        except Exception as e:
            rospy.logerr(f"Error during YOLO processing: {e}")
    
    #충돌 확률 계산 알고리줌 - 최고 확률 반환 
    def compute_highest_collision_probability(self, predicted_boxes, frame):
        height, width, _ = frame.shape
        center_x, center_y = width//2, int(height *0.66)
        if self.mission_info.mission_num == 7 :
            roi_x1 = int(width* 0.1)
            roi_x2 = int(width * 0.9)
            roi_y1 = int(height * 0.35 )
            roi_y2 = int(height * 0.9)
        else : 
            roi_x1, roi_y1, roi_x2, roi_y2 = self.set_roi_by_mission(self.mission_info.mission_num, width, height)
       
        #특정 roi로 자르기 
        roi_area = self.calculate_box_area(roi_x1, roi_y1, roi_x2, roi_y2)
        highest_collision_probability = 0
        best_classnum = 888  # 초기값: classnum

        for object_id, box, conf, cls in predicted_boxes:
            x1, y1, x2, y2 = map(int, box)

            #박스의 4꼭짓점 중 roi 안으로 들어온 부분 
            overlap_x1 = max(x1, roi_x1)
            overlap_y1 = max(y1, roi_y1)
            overlap_x2 = min(x2, roi_x2)
            overlap_y2 = min(y2, roi_y2)
            
            # 바운딩 박스가 ROI 내에서 차지하는 비율 계산
            box_area = self.calculate_box_area(x1, y1, x2, y2)
            occupancy_ratio = box_area / roi_area
            if occupancy_ratio >= 1.4:
                best_classnum = int(cls.item())
                return best_classnum, 100.0
                
            if overlap_x1 < overlap_x2 and overlap_y1 < overlap_y2:
                
                overlap_area = self.calculate_box_area(overlap_x1, overlap_y1, overlap_x2, overlap_y2)
                box_iou = overlap_area / box_area  # IoU 기반으로 겹치는 영역 비율 계산


                # 바운딩 박스 중심점과 이미지 중심점 사이의 거리 비율 계산
                closest_point = self.get_closest_point_to_center(x1, y1, x2, y2, center_x, center_y)
                distance_ratio = self.calculate_distance_ratio(closest_point[0], closest_point[1], center_x, center_y, width, height)
                
                
                #박스 크기 변화율이 클 수록 충돌 확률이 높음 
                area_change_rate = self.calculate_area_change_rate(object_id, box_area)
                
                if (distance_ratio <= 0.05 or box_iou >= 0.95):
                    collision_probability = (4.5 * (1 - distance_ratio) + 3.5 * box_iou + 1.5 * area_change_rate) * occupancy_ratio / 9.5 * 100.0 
                elif (self.mission_info.mission_num == 51):
                    collision_probability = (3.5 * (1 - distance_ratio) + 4.5 * box_iou) * occupancy_ratio / 8 * 100.0    
                else : 
                    collision_probability = (4.5 * (1 - distance_ratio) + 3.5 * box_iou) * occupancy_ratio / 8 * 100.0 
                
                if (self.mission_info.mission_num == 5):
                        collision_probability = (1.5 * (1 - distance_ratio) + 3.0 * box_iou)/ 4.5 * 100.0    
                if self.obstacle.classnum==0 or self.obstacle.classnum==1 or self.obstacle.classnum==3 :
                        collision_probability = (4.5 * (1 - distance_ratio) + 2.0 * box_iou ) * 13 * occupancy_ratio / 6.5 * 100.0
                collision_probability = max(0.0, min(collision_probability, 100))  # 0에서 100 사이로 제한
                
                if collision_probability > highest_collision_probability:
                    highest_collision_probability = collision_probability
                    best_classnum = int(cls.item())


        return best_classnum, highest_collision_probability 
    
    # 신뢰도 낮은 것과 class_number가 0, 1, 2 3 가 아닌 것을 버리기
    def filter_results_by_confidence(self, results):
        allowed_classes = {0, 1, 2, 3, 888}  # 허용된 클래스 번호 집합
        filtered_results = []
        for result in results:
            filtered_boxes = [box for box in result.boxes if box.conf > self.confidence_threshold and int(box.cls.item()) in allowed_classes]
            filtered_results.append(filtered_boxes)
        return filtered_results

    #이미지 받아오기 콜백 
    def image_callback(self, data):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            self.latest_frame = frame
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert image: {e}")
    
    #mission number 수신용 콜백함수 
    def mission_callback(self, msg):
        try:
            self.mission_info = msg
        except ValueError as e:
            rospy.logerr(f"Invalid driving mode value: {self.mission_info.mission_num}")
    
    #mission number 에 따라 다르게 동작하는 로직  
    def timer_callback(self, event): 
        if self.mission_info.mission_num in [3,6]:#주차 및 gpsshaded에서 완전 끄기 
            return
        if self.latest_frame is not None:
            if self.mission_info.mission_num in [0,1,5,51]:  # 끼어들기, 차간간격
                self.calculate_collision_probability(self.latest_frame)
            
            elif self.mission_info.mission_num in [2,4,6,7]:  # 동적 장애물 감지
                self.calculate_collision_probability(self.latest_frame)
            
            elif self.mission_info.mission_num == 71:  # 신호등 탐지 미션
                self.calculate_collision_probability(self.latest_frame.copy())
                # ROI 설정
                roi_x1, roi_y1, roi_x2, roi_y2 = self.set_roi_by_mission(self.mission_info.mission_num, 
                                                                         self.latest_frame.shape[1], 
                                                                         self.latest_frame.shape[0])
                frame = self.latest_frame.copy()
                roi= frame[roi_y1:roi_y2,roi_x1:roi_x2]
                # 신호등 검출 및 색상 분석
                self.detect_traffic_light_color(roi)
            else: 
                self.calculate_collision_probability(self.latest_frame)

            self.latest_frame = None
    
    def visualize_and_publish_roi(self, frame, roi_x1, roi_y1, roi_x2, roi_y2):
        """
        현재 설정된 ROI를 이미지에 시각화하고 퍼블리시하는 함수
        """
        # ROI를 시각화 (이미지에 사각형 그리기)
        visualized_frame = frame.copy()
        cv2.rectangle(visualized_frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (20, 20, 20), 2)
        
        try:
            # 이미지를 ROS 메시지로 변환
            roi_image_msg = self.bridge.cv2_to_imgmsg(visualized_frame, "bgr8")
            
            # 퍼블리시
            self.roi_image_pub.publish(roi_image_msg)
            rospy.loginfo("Published ROI visualized image.")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert ROI visualized image: {e}")

        
def main():
    yolo_node = YoloNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLO node.")

if __name__ == '__main__':
    main()

    ##waw