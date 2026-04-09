from fairino import Robot
import time

class FairinoCommand():
    def __init__(self, robot_ip='192.168.58.2'):
        self.robot = Robot.RPC(robot_ip)

    def set_speed(self,speed):
        self.robot.SetSpeed(speed)
        
    def move_cart(self, mode=1, x=0.0, y=0.0, z=0.0, mm=0):
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

    def move_joint(self, joint, tool=0, user=0):
        joint_list = [joint[i] for i in range(len(joint))]
        ret = self.robot.MoveJ(joint_list, tool, user)
        print(f"MoveJ to {joint_list}, ret : {ret}")

    def gripper_activate(self, claw_num=1, action=1):
        ret,gripper_config = self.robot.GetGripperConfig()
        self.robot.SetGripperConfig(gripper_config[1],gripper_config[2])
        time.sleep(1)
        self.robot.ActGripper(claw_num,action)
        time.sleep(1)
        print(f"gripper activate : {ret}")

    def move_gripper(self, claw_num=1, pos=0, speed=100, force=50, maxtime=30000, block=0):
        time.sleep(1)
        self.robot.MoveGripper(claw_num,pos,speed,force,maxtime,block) # open gripper
        print(f"move gripper, pos : {pos}")
        time.sleep(1)
        self.robot.GetGripperMotionDone()
        time.sleep(1)
