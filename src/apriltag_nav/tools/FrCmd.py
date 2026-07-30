import sys
import os
import time
import numpy as np
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FAIRINO_PATH = os.path.join(
    BASE_DIR,
    "../../fairino_sdk/fairino-python-sdk/Linux"
)
sys.path.append(FAIRINO_PATH)

from fairino import Robot

class FairinoCommand():
    def __init__(self, robot_ip='192.168.58.2'):
        self.robot = Robot.RPC(robot_ip)

    def set_speed(self,speed):
        """
            speed : int(1~100)
        """
        self.robot.SetSpeed(speed)
        
    def move_linear(self, mode, x, y, z, mm=0):
        """
            mode : 0, 1, 2
            0 : abs base coord
            1 : base coord
            2 : TCP coord
            
            x, y, z : float(0.0 ~ 1.0)
        """
        n_pos = [x, y, z, 0.0, 0.0, 0.0]   
        error,_ = self.robot.GetActualTCPPose()
        count = mm
        error = self.robot.ServoMoveStart()  #Servo motion start
        while(count):
            error = self.robot.ServoCart(mode, n_pos)   # Cartesian space servo mode motion
            count = count - 1
            time.sleep(0.008)
        error = self.robot.ServoMoveEnd()  #Servo motion end
        print(f"Move Cartesian : {error}")

    def move_point(self, desc_pos):
        """
            desc_pos = [x, y, z, rx, ry, rz]
        """
        self.robot.MoveCart(desc_pos, tool=0, user=0)
        
    def move_joint(self, joint, tool=0, user=0):
        """
            joint : [J1, J2, J3, J4, J5, J6]
        """
        joint_list = [joint[i] for i in range(len(joint))]
        ret = self.robot.MoveJ(joint_list, tool, user)
        print(f"MoveJ to {joint_list}, ret : {ret}")

    def move_circle(self, desc_posc1, desc_posc2):
        """
            desc_posc1 : Path point
            desc_posc2 : Target point
            tool = 1, user = 0 -> 0,0
        """
        
        ret = self.robot.MoveC(desc_posc1, 1, 0, desc_posc2, 1, 0)  #Circular arc motion in Cartesian space
        print(f"MoveC, ret : {ret}")
        
    def gripper_activate(self, claw_num=1, action=1):
        ret,gripper_config = self.robot.GetGripperConfig()
        self.robot.SetGripperConfig(gripper_config[1],gripper_config[2])
        time.sleep(1)
        self.robot.ActGripper(claw_num,action)
        time.sleep(1)
        print(f"gripper activate : {ret}")

    def move_gripper(self, claw_num=1, position=0, speed=100, force=50, maxtime=30000, block=0):
        self.robot.MoveGripper(claw_num,position,speed,force,maxtime,block)
        print(f"move gripper, position : {position}")
        time.sleep(0.5)
        self.robot.GetGripperMotionDone()
        time.sleep(0.5)

    def get_inverse_kin(self, point):
        """
            point : [x, y, z, rx, ry, rz]
        """
        result = self.robot.GetInverseKin(0,point,config=-1)
        return result
    
    def get_foward_kin(self, joint):
        """
            joint : [J1, J2, J3, J4, J5, J6]
        """
        result = self.robot.GetForwardKin(joint)
        return result
    
    def get_tcp_position(self):
        """

        Returns:
            [tuple]: [0,[x, y, z, rx, ry, rz]]
            ret[1] : [x, y, z, rx, ry, rz]
        """
        ret = self.robot.GetActualTCPPose()

        return ret[1]
