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
    else:
        rospy.init_node('rrt')

    boxes = np.array([
        # obstacle
        [0.4, 0, 0.25, 0, 0, 0, 0.3, 0.05, 0.5],
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
        '''
        TODO: Implement constraint function and its gradient. 
        
        This constraint should enforce the end-effector stays upright.
        Hint: Use the roll and pitch angle in desired_ee_rp. The end-effector is upright in its home state.

        Input:
            q - a joint configuration

        Output:
            err - a non-negative scalar that is 0 when the constraint is satisfied
            grad - a vector of length 6, where the ith element is the derivative of err w.r.t. the ith element of ee
        '''
        ee = fr.ee(q)

        err, grad = None, None
        return err, grad

    joints_start = fr.home_joints.copy()
    joints_start[0] = -np.deg2rad(45)
    joints_target = joints_start.copy()
    joints_target[0] = np.deg2rad(45)

    rrtc = RRTConnect(fr, is_in_collision)
    constraint = None # ee_upright_constraint
    plan = rrtc.plan(joints_start, joints_target, constraint)
    
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
        forward_plan = plan[::5]
        backward_plan = forward_plan[::-1]

        while True:
            for joints in tqdm(forward_plan):
                fa.goto_joints(joints, duration=0.5)
            sleep(1)
            for joints in tqdm(backward_plan):
                fa.goto_joints(joints, duration=0.5)
