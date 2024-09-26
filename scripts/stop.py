import rospy
from std_msgs.msg import Int16, Bool
from mando.msg import mission

class Stopsignal:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.Subscriber("/traffic_light_color", Int16, self.traffic_callback)
        rospy.Subscriber("/mission", mission, self.mission_callback)

        self.stop_pub = rospy.Publisher('/stop_signal', Bool, queue_size=1)
        self.to_stop = False

        self.traffic_color = 0
        self.mission_info = mission()

        self.is_traffic = False
        self.is_mission = False

        rate = rospy.Rate(15)  # 30Hz
        while not rospy.is_shutdown():
            if self.is_mission:
                # 특정 미션 구간에서 일단 정지
                if self.mission_info.mission_num in [2, 6, 10]:
                    self.to_stop = True
                    if self.traffic_color == 1:
                        # 신호등이 빨간불일 때 계속 정지
                        self.to_stop = True
                    elif self.traffic_color ==3:
                        # 신호등이 초록불일 때 정지 해제
                        self.to_stop = False
                else:
                    # 해당 미션 구간이 아니면 정지 신호 해제
                    self.to_stop = False

                # 정지 신호를 퍼블리시
                
                self.stop_pub.publish(self.to_stop)
                rospy.loginfo(f"to stop: {self.to_stop}")

            rate.sleep()

    def traffic_callback(self, msg):
        try:
            self.traffic_color = msg.data
            self.is_traffic = True
        except ValueError as e:
            rospy.logerr(f"Invalid traffic: {self.traffic_color}")

    def mission_callback(self, msg):
        try:
            self.mission_info = msg
            self.is_mission = True
        except ValueError as e:
            rospy.logerr(f"Invalid mission value: {self.mission_info.mission_num}")

if __name__ == '__main__':
    try:
        Stopsignal()
    except rospy.ROSInterruptException:
        pass
