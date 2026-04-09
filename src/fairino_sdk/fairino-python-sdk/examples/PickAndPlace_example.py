import time
from FrCmd import FairinoCommand

class FrScript:
    def __init__(self, ip) -> None:
        self.FrCmd = FairinoCommand(ip)
        self.FrCmd.gripper_activate(1,1)
        self.FrCmd.set_speed(30)
        time.sleep(1)

    def main(self):
        #pick
        self.FrCmd.move_joint([0,-45,-135,-90,90,0])
        self.FrCmd.move_cart(1, 0.0, 0.0, -1.0, 100)
        self.FrCmd.move_gripper(1,0)
        self.FrCmd.move_cart(1, 0.0, 0.0, 1.0,100)

        #place
        self.FrCmd.move_joint([45,-45,-135,-90,90,0])
        self.FrCmd.move_cart(1, 0.0, 0.0, -1.0,100)
        self.FrCmd.move_gripper(1,100)
        self.FrCmd.move_cart(1, 0.0, 0.0, 1.0,100)

if __name__ == "__main__":
    frscript = FrScript('192.168.58.2')
    while True:
        frscript.main()