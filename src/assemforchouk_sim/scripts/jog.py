#!/usr/bin/env python3
"""
Send one axis to a position.

    ros2 run assemforchouk_sim jog.py lift 1.5
    ros2 run assemforchouk_sim jog.py carriage_slide -1.2 --time 4
"""
import argparse, sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

JOINTS = ['carriage_slide', 'pusher_extend', 'lift', 'shelf_extend']
LIMITS = {'carriage_slide': (-1.90, 3.40), 'pusher_extend': (0.0, 1.05),
          'lift': (-2.05, 4.10), 'shelf_extend': (-0.15, 1.15)}


class Jog(Node):
    def __init__(self):
        super().__init__('assemforchouk_jog')
        self.state = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.cli = ActionClient(self, FollowJointTrajectory,
                                '/cell_controller/follow_joint_trajectory')

    def _cb(self, msg):
        self.state = dict(zip(msg.name, msg.position))

    def current(self, timeout=10.0):
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while rclpy.ok() and self.state is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.state or {}

    def go(self, joint, value, seconds):
        lo, hi = LIMITS[joint]
        if not lo <= value <= hi:
            raise SystemExit(f"'{joint}' limit is [{lo}, {hi}], got {value}")
        cur = self.current()
        pos = [cur.get(j, 0.0) for j in JOINTS]
        pos[JOINTS.index(joint)] = value
        if not self.cli.wait_for_server(timeout_sec=20.0):
            raise SystemExit('cell_controller action server not available')
        traj = JointTrajectory(joint_names=JOINTS)
        pt = JointTrajectoryPoint(positions=[float(p) for p in pos],
                                  velocities=[0.0] * len(JOINTS))
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        print('  '.join(f'{n}={p:+.3f}' for n, p in zip(JOINTS, pos)))
        fut = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        h = fut.result()
        if h is None or not h.accepted:
            raise SystemExit('goal rejected')
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        print('done, error_code =', res.result().result.error_code)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('joint', choices=JOINTS)
    ap.add_argument('value', type=float)
    ap.add_argument('--time', type=float, default=5.0, help='seconds to get there')
    a = ap.parse_args([x for x in sys.argv[1:] if not x.startswith('--ros-args')])
    rclpy.init()
    try:
        Jog().go(a.joint, a.value, a.time)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
