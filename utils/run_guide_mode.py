from frankapy import FrankaArm


if __name__ == '__main__':
    print('Starting robot')
    fa = FrankaArm()

    input('Click [Enter] to start guide mode for 10s.')

    print('Applying 0 force torque control for {}s'.format(10))
    fa.apply_effector_forces_torques(10, 0, 0, 0)

    pose = fa.get_pose()
    joints = fa.get_joints()

    print('position is: {}'.format(repr(pose.translation)))
    print('joints are: {}'.format(repr(joints)))
