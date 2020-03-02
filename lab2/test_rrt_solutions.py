import argparse
import numpy as np
from time import sleep
import rospy
from tqdm import tqdm
from frankapy import FrankaArm

from franka_robot import FrankaRobot 
from collision_boxes_publisher import CollisionBoxesPublisher
from rrt_connect import RRTConnect


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run_on_robot', action='store_true')
    parser.add_argument('--seed', '-s', type=int, default=0)
    args = parser.parse_args()

    np.random.seed(args.seed)
    fr = FrankaRobot()

    if args.run_on_robot:
        fa = FrankaArm()
        fa.close_gripper()
    else:
        rospy.init_node('rrt')

    boxes = np.array([
        # obstacle in hw2
        # [0.4, 0, 0.25, 0, 0, 0, 0.3, 0.05, 0.5],
        # obstacle in lab2
        [0.41, 0.0065, 0.122, 0, 0, 0, 0.3, 0.107, 0.244],
        # sides
        [0.15, 0.46, 0.5, 0, 0, 0, 1.2, 0.01, 1.1],
        [0.15, -0.46, 0.5, 0, 0, 0, 1.2, 0.01, 1.1],
        # back
        [-0.41, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
        # front
        [0.75, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
        # top
        [0.2, 0, 1, 0, 0, 0, 1.2, 1, 0.01],
        # bottom
        [0.2, 0, -0.05, 0, 0, 0, 1.2, 1, 0.01]
    ])
    def is_in_collision(joints):
        for box in boxes:
            if fr.check_box_collision(joints, box):
                return True
        return False

    desired_ee_rp = fr.ee(fr.home_joints)[3:5]
    def ee_upright_constraint(q):
        ee_rp = fr.ee(q)[3:5]
        diff = ee_rp - desired_ee_rp

        err = diff @ diff

        grad = np.zeros(6)
        grad[3:5] = 2 * diff
        return err, grad

    # hw2 start and target joints
    # joints_start = fr.home_joints.copy()
    # joints_start[0] = -np.deg2rad(45)
    # joints_target = joints_start.copy()
    # joints_target[0] = np.deg2rad(45)

    # lab2 start and target joints
    joints_start = np.array([-0.4220219, 0.01425756, -0.20841136, -2.58497032, 0.10319939, 2.51418879, 0.10520479])
    joints_target = np.array([-0.07483001, 0.18115084, 0.69566339, -2.47841354, -0.10288276, 2.63124746, 1.48171695])

    rrt = RRTConnect(fr, is_in_collision)
    constraint = ee_upright_constraint
    plan = rrt.plan(joints_start, joints_target, None)
    
    i = 0
    collision_boxes_publisher = CollisionBoxesPublisher('collision_boxes')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        joints = plan[i % len(plan)]
        fr.publish_joints(joints)
        fr.publish_collision_boxes(joints)
        collision_boxes_publisher.publish_boxes(boxes)

        if args.run_on_robot:
            if i > len(plan) * 3:
                while True:
                    inp = input('Would you like to [c]ontinue to execute the plan or [r]eplay the plan? ')
                    if len(inp) == 1 and inp in 'cr':
                        break
                    print('Please enter a valid input! Only c and r are accepted!')
                if inp == 'r':
                    i = 0
                else:
                    break

        i += 1
        rate.sleep()

    if args.run_on_robot:
        while True:
            input('Press [Enter] to run guide mode for 10s and move robot to near the strat configuration.')
            fa.apply_effector_forces_torques(10, 0, 0, 0)

            while True:
                inp = input('Would you like to [c]ontinue or [r]erun guide mode? ')
                if len(inp) == 1 and inp in 'cr':
                    break
                print('Please enter a valid input! Only c and r are accepted!')

            if inp == 'c':
                break

        print('Running plan...')
        fa.goto_joints(joints_start)
        forward_plan = plan[::4]
        backward_plan = forward_plan[::-1]

        while True:
            for joints in tqdm(forward_plan):
                fa.goto_joints(joints, duration=max(float(max(joints - fa.get_joints()) / 0.1), 1))
                sleep(0.1)
            sleep(1)
            for joints in tqdm(backward_plan):
                fa.goto_joints(joints, duration=max(float(max(joints - fa.get_joints()) / 0.1), 1))
                sleep(0.1)
