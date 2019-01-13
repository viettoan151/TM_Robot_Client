from tm_robot import TM5_Robot

if __name__=='__main__':
    robot = TM5_Robot([0],'192.168.0.10',6188)
    robot.test_robot()