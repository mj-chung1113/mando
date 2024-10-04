#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage, Image
from mando.msg import mission, obstacle  # 사용자 정의 메시지 임포트 
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16

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
        
        self.bridge = CvBridge()
        self.roi_image_pub = rospy.Publisher("/roi_visualization", Image, queue_size=5)  # ROI 시각화 퍼블리셔 추가
       
        # 중앙선 검출 시각화를 위한 퍼블리셔
        self.lane_image_pub = rospy.Publisher("/lane_detection_visualization", Image, queue_size=5)

        # GPU 사용 설정
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.path = 'yolov8n.pt'
        self.model = YOLO(self.path)
        self.model = self.model.to(self.device)
        
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
 
        if mission_num == 2 or mission_num == 6: 
            # 신호등 
            roi_x1 = int(frame_width * 0.44)
            roi_x2 = int(frame_width * 0.56)
            roi_y1 = int(frame_height * 0.30)
            roi_y2 = int(frame_height * 0.50)
        
        elif mission_num not in [2,6]:
            # 동적장애물
            roi_x1 = int(frame_width)
            roi_x2 = int(frame_width)
            roi_y1 = int(frame_height * 0.35)
            roi_y2 = int(frame_height * 0.85)
        
        else:
            # 기본 ROI 설정: 이미지 전체 영역
            roi_x1 = int(frame_width * 0.05)
            roi_x2 = int(frame_width * 0.95)
            roi_y1 = int(frame_height * 0.35)
            roi_y2 = int(frame_height * 0.85)
        
        if self.latest_frame is not None:
            self.visualize_and_publish_roi(self.latest_frame, roi_x1, roi_y1, roi_x2, roi_y2)
        return roi_x1, roi_y1, roi_x2, roi_y2
    
    def detect_lanes(self, frame):
        # 이미지를 그레이스케일로 변환
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 가우시안 블러로 노이즈 제거
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 캐니 엣지 검출기 사용
        edges = cv2.Canny(blurred, 50, 150)

        # 중앙선(노란색) 검출
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([16, 91, 165])
        yellow_upper = np.array([64, 209, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        yellow_edges = cv2.bitwise_and(edges, yellow_mask)

        lines_yellow = cv2.HoughLinesP(yellow_edges, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=150)
        if lines_yellow is not None:
            for line in lines_yellow:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 노란선을 검출하여 노란색으로 그림

        # 버드아이 뷰에서 차선 검출 결과를 ROS 메시지로 변환 및 퍼블리시
        try:
            lane_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.lane_image_pub.publish(lane_image_msg)
            rospy.loginfo("Published lane detection visualization from Bird's Eye View.")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert lane detection image: {e}")

            
    def apply_birds_eye_view(self, frame):
        """
        버드와이드 뷰(Bird's Eye View) 변환을 적용하는 함수
        """
        height, width = frame.shape[:2]

        # 원본 이미지의 4개 포인트 (이 포인트는 차량 앞쪽의 사각형 영역을 선택)
        src_points = np.float32([[width*0.1, height*0.9],   # 좌측 하단
                                [width*0.9, height*0.9],   # 우측 하단
                                [width*0.7, height*0.52],   # 우측 상단
                                [width*0.3, height*0.52]])  # 좌측 상단

        # 변환 후의 출력 이미지에서 원하는 4개 포인트
        dst_points = np.float32([[0, height],               # 좌측 하단
                                [width, height],           # 우측 하단
                                [width, 0],                # 우측 상단
                                [0, 0]])                   # 좌측 상단

        # 변환 행렬 계산
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        # 원근 변환 적용
        birds_eye_view = cv2.warpPerspective(frame, matrix, (width, height))

        return birds_eye_view

    def image_callback(self, data):
        try:
            # 압축 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

            # 이미지 유효성 확인
            if cv_image is None or cv_image.size == 0:
                rospy.logerr("Received an invalid image from cv_bridge.")
                return
        
            self.latest_frame = cv_image.copy()

            # 버드아이 뷰 적용
            birds_eye_view_frame = self.apply_birds_eye_view(self.latest_frame)

            # 버드아이 뷰에서 차선 및 중앙선 검출
            self.detect_lanes(birds_eye_view_frame)

            # 변환된 이미지 (버드아이 뷰) 퍼블리시
            try:
                bev_image_msg = self.bridge.cv2_to_imgmsg(birds_eye_view_frame, "bgr8")
                self.lane_image_pub.publish(bev_image_msg)  # rqt_image_view에서 확인 가능
                rospy.loginfo("Published Bird's Eye View lane detection visualization.")
            except CvBridgeError as e:
                rospy.logerr(f"Could not convert Bird's Eye View image: {e}")

        except CvBridgeError as e:
            rospy.logerr(f"Could not convert image: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error in image_callback: {e}")
    

         

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
    
    #mission number 수신용 콜백함수 
    def mission_callback(self, msg):
        try:
            self.mission_info = msg
        except ValueError as e:
            rospy.logerr(f"Invalid driving mode value: {self.mission_info.mission_num}")
    
    #mission number 에 따라 다르게 동작하는 로직  
    def timer_callback(self, event): 
        
        if self.latest_frame is not None:
            roi_x1, roi_y1, roi_x2, roi_y2 = self.set_roi_by_mission(self.mission_info.mission_num, 
                                                                            self.latest_frame.shape[1], 
                                                                            self.latest_frame.shape[0])
            frame = self.latest_frame.copy()
            roi= frame[roi_y1:roi_y2,roi_x1:roi_x2]
            if self.mission_info.mission_num in [2,6]:  # 신호등 탐지 미션
                # ROI 설정
                roi_x1, roi_y1, roi_x2, roi_y2 = self.set_roi_by_mission(self.mission_info.mission_num, 
                                                                            self.latest_frame.shape[1], 
                                                                            self.latest_frame.shape[0])
                frame = self.latest_frame.copy()
                roi= frame[roi_y1:roi_y2,roi_x1:roi_x2]
                # 신호등 검출 및 색상 분석
                self.detect_traffic_light_color(roi)
        else: 
            return

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
        rospy.spin()  # ROS 노드를 실행하여 콜백을 유지합니다.
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLO node.")


if __name__ == '__main__':
    main()
