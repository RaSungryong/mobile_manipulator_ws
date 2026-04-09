#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Bool
from woon_controller.msg import Pose2DWithFlag
from fairino import Robot

class MoldManipulatorNode:
    def __init__(self):
        rospy.init_node('mold_manipulator_node')

        # 1) 금형 표면 타겟 글로벌 좌표 (m)
        self.mold_target_global = np.array([0.84, -1.015, 1.15])

        # 2) 매니퓰레이터 오프셋: 모바일 베이스 링크에서 TCP까지 (m)
        #    z축에만 1.2m 올려진다고 가정
        self.manipulator_offset = np.array([0.0, 0.0, 0.8])

        # Fairino RPC 클라이언트
        self.robot = Robot.RPC('192.168.58.2')
        self.tool = 0
        self.user = 0

        # 초기 자세로 복귀
        self.initial_joints = [-90, -90, 90, -90, -90, 0]
        self.robot.MoveJ(self.initial_joints, self.tool, self.user, vel=100)

        # 로봇의 월드 좌표계 위치(gt)

        # 토픽 구독/퍼블리시
        rospy.Subscriber('/robot_pose2d', Pose2DWithFlag, self.callback)
        self.done_pub = rospy.Publisher('/scan_finished', Bool, queue_size=1)

        rospy.spin()

    def callback(self, msg: Pose2DWithFlag):
        if not msg.flag:
            return
        rospy.loginfo(f"mobile_robot: {msg}")
        # 모바일 로봇 베이스 글로벌 좌표 (m)
        base_x, base_y = msg.x, msg.y
        base_z = 0.0
        mobile_base_global = np.array([base_x, base_y, base_z])


        # 3) 매니퓰레이터 글로벌 좌표 계산
        manipulator_global = mobile_base_global + self.manipulator_offset

        # 4) 상대 타겟 좌표 계산
        relative_target = self.mold_target_global - manipulator_global

        # 5) mm 단위 + orientation (roll,pitch,yaw)
        #    orientation은 그대로 금형 타겟의 yaw(0deg)로 설정
        desc_pos = [
            -relative_target[0] * 1000,
            -relative_target[1] * 1000,
            relative_target[2] * 1000,
            180.0,  # roll
            0.0,  # pitch
            0.0,  # yaw
        ]

        rospy.loginfo(f"Moving to relative target (mm,deg): {desc_pos}")
        ret = self.robot.MoveCart(desc_pos, self.tool, self.user, vel=100)
        error,local_pos = self.robot.GetActualTCPPose()
        print("GetActualTCPPose",error,local_pos)

        if ret == 0:
            rospy.loginfo("MoveCart succeeded")
        else:
            rospy.logerr(f"MoveCart failed, code: {ret}")

        # self.robot.MoveJ(self.initial_joints, self.tool, self.user, vel=100)
        # 완료 신호
        self.done_pub.publish(True)

if __name__ == '__main__':
    try:
        MoldManipulatorNode()
    except rospy.ROSInterruptException:
        pass