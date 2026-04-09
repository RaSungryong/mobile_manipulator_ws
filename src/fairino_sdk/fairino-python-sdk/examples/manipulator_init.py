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
        self.manipulator_offset = np.array([0.0, 0.0, 0.8])

        # Fairino RPC 클라이언트
        self.robot = Robot.RPC('192.168.58.2')
        self.tool = 0
        self.user = 0

        # 초기 자세로 복귀
        self.initial_joints = [-90, -90, 90, -90, -90, 0]
        self.robot.MoveJ(self.initial_joints, self.tool, self.user, vel=100)

        # /robot_pose2d 구독
        rospy.Subscriber('/robot_pose2d', Pose2DWithFlag, self.callback)
        self.done_pub = rospy.Publisher('/scan_finished', Bool, queue_size=1)

        rospy.spin()

    def callback(self, msg: Pose2DWithFlag):
        # flag=False 이면 무시
        if not msg.flag:
            return

        # 1) 모바일 로봇 베이스 글로벌 좌표 (m)
        base_x, base_y = msg.x, msg.y
        base_z = 0.0
        mobile_base_global = np.array([base_x, base_y, base_z])

        # 2) 매니퓰레이터 베이스 글로벌 좌표 계산
        manipulator_global = mobile_base_global + self.manipulator_offset

        # 3) 로컬 TCP 포즈 읽어오기 (mm, deg)
        err, local_tcp = self.robot.GetActualTCPPose()
        if err != 0:
            rospy.logerr(f"GetActualTCPPose error: {err}")
            return

        # local_tcp = [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]
        local_pos_m = np.array(local_tcp[:3]) / 1000.0  # → m
        local_rpy = local_tcp[3:]                      # → deg

        # 4) 태그의 월드 좌표 계산
        #    world_tag_pos = world_manipulator_base + local_tcp_pose
        tag_world_pos = manipulator_global + local_pos_m

        # 5) 로그로 출력
        rospy.loginfo(f"→ Manipulator base (world): {manipulator_global}")
        rospy.loginfo(f"→ TCP local pos (m):       {local_pos_m}, rpy(deg): {local_rpy}")
        rospy.loginfo(f"→ Tag world pos (m):      {tag_world_pos}")

        # (옵션) 금형 target과 비교
        rel_to_tag = self.mold_target_global - tag_world_pos
        rospy.loginfo(f"→ Error to mold target (m): {rel_to_tag}")

        # 완료 신호
        self.done_pub.publish(True)

if __name__ == '__main__':
    try:
        MoldManipulatorNode()
    except rospy.ROSInterruptException:
        pass
