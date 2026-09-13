#!/usr/bin/env python3
"""
Automatic storage / retrieval work cycle for the Assemforchouk cell.

Drives the four prismatic axes through a full put-away and retrieval sequence
via the cell_controller JointTrajectory action, then repeats.

    ros2 run assemforchouk_sim cycle_demo.py
    ros2 run assemforchouk_sim cycle_demo.py --ros-args -p loops:=3 -p speed:=1.5
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ['carriage_slide', 'pusher_extend', 'lift', 'shelf_extend']

# ---------------------------------------------------------------- the cycle
#
# How the cell works:
#
#   1. The carriage runs LEFT along the roller table to the infeed station and
#      RECEIVES a package into its open channel.
#   2. It carries the package RIGHT, stopping with its channel lined up on one
#      of the comptoir's slots.
#   3. The shelf comes alongside on the conveyor-side chain run.
#   4. The pusher strokes across (+Y) and RELEASES the package onto the shelf.
#   5. The shelf then CIRCULATES the whole paternoster loop - up the conveyor
#      side, over the top sprocket, down the far side, under the bottom sprocket
#      and back up - arriving at the receive level ready for the next package.

import math
import time

INFEED_SLIDE = -1.03        # carriage at the infeed end, lined up on slot 1
SHELF_TRANSFER = 1.055      # shelf deck abuts the carriage deck edge to edge
PUSH_STROKE = 1.05          # carries the package clear of the seam and onto
                            # the middle of the shelf deck

# carriage_slide values that line the carriage channel up on each comptoir slot
SLOT_SLIDE = [-1.03, -0.71, -0.39, -0.07, 0.25, 0.57, 0.89, 1.21, 1.53, 1.85, 2.17, 2.487]

# Slots 1-5 arrive pre-loaded (see spawn_payload.py), so put-away targets the
# free ones.  Aiming at an occupied slot stacks one package on another and both
# end up on the floor.
N_PRELOADED = 5
FREE_SLOTS = list(range(N_PRELOADED, len(SLOT_SLIDE)))

# ---- the chain loop, measured off base_chain.stl / base_drum.stl -------------
# joint values: shelf_extend q_y = -3.818 - Y ,  lift q_z = Z + 1.458
#
# The conveyor-side run is measured at 1.042 and the pose where the shelf deck
# abuts the carriage is 1.055.  They are the same place to within 13 mm, so the
# loop uses the abutting value for both: the carrier receives, circulates, and
# comes back to exactly where it started.  No stepping in and out.
RUN_CONVEYOR = SHELF_TRANSFER   # conveyor-side run == the receive pose
RUN_FAR = -0.038            # q_y of the far-side vertical chain run
WRAP_MID = (RUN_CONVEYOR + RUN_FAR) / 2.0
WRAP_DY = (RUN_CONVEYOR - RUN_FAR) / 2.0    # 0.540, the sprocket radius in q_y
Q_TOP = 3.495               # q_z of the top sprocket centre
Q_BOT = -1.499              # q_z of the bottom sprocket centre
WRAP_TOP = 0.527            # top wrap radius in q_z
WRAP_BOT = 0.460            # bottom wrap, flattened slightly to clear the floor
ARC_STEPS = 9


def paternoster_loop(slide, t0=0.0, rise=7.0, wrap=5.0, fall=7.0):
    """One full circulation of the carrier, as timed (positions, t) waypoints.

    Starts and ends at the receive level on the conveyor-side run.
    """
    pts, t = [], t0

    def add(qy, qz, dt):
        nonlocal t
        t += dt
        pts.append(([slide, 0.0, qz, qy], t))

    add(RUN_CONVEYOR, Q_TOP, rise)              # up the conveyor side
    for i in range(1, ARC_STEPS + 1):           # over the top sprocket
        a = math.pi * i / ARC_STEPS
        add(WRAP_MID + WRAP_DY * math.cos(a), Q_TOP + WRAP_TOP * math.sin(a),
            wrap / ARC_STEPS)
    add(RUN_FAR, Q_BOT, fall)                   # down the far side
    for i in range(1, ARC_STEPS + 1):           # under the bottom sprocket
        a = math.pi + math.pi * i / ARC_STEPS
        add(WRAP_MID + WRAP_DY * math.cos(a), Q_BOT + WRAP_BOT * math.sin(a),
            wrap / ARC_STEPS)
    add(RUN_CONVEYOR, 0.0, rise * 0.35)         # back up to the receive level
    return pts


def build_cycle(slot_slide):
    """(label, carriage_slide, pusher_extend, lift, shelf_extend, seconds)"""
    # shelf_extend stays at the receive pose throughout: the carrier waits in
    # place for the carriage instead of stepping out and back between packages.
    return [
        ('carriage -> infeed (left)     ', INFEED_SLIDE, 0.00, 0.00, SHELF_TRANSFER, 6.0),
        ('RECEIVE package                ', INFEED_SLIDE, 0.00, 0.00, SHELF_TRANSFER, 2.5),
        ('carry it right to the slot     ', slot_slide,   0.00, 0.00, SHELF_TRANSFER, 7.0),
        ('PUSH - release onto the shelf  ', slot_slide, PUSH_STROKE, 0.00, SHELF_TRANSFER, 4.5),
        ('pusher retracts                ', slot_slide,   0.00, 0.00, SHELF_TRANSFER, 3.0),
    ]


class CycleDemo(Node):
    def __init__(self):
        super().__init__('assemforchouk_cycle_demo')
        self.declare_parameter('loops', 0)        # 0 = run forever
        self.declare_parameter('speed', 1.0)      # >1 = faster
        self.declare_parameter('controller', 'cell_controller')
        # hold still long enough for spawn_payload to finish loading the shelf;
        # moving first would slide the deck out from under the new packages
        self.declare_parameter('start_delay', 7.0)
        self.speed = max(0.1, self.get_parameter('speed').value)
        self.loops = self.get_parameter('loops').value
        name = self.get_parameter('controller').value
        self.cli = ActionClient(self, FollowJointTrajectory,
                                f'/{name}/follow_joint_trajectory')
        self.get_logger().info(f'waiting for /{name}/follow_joint_trajectory ...')
        if not self.cli.wait_for_server(timeout_sec=120.0):
            raise SystemExit(f'{name} action server never appeared')
        self.get_logger().info('controller ready - starting work cycle')

    def send(self, label, positions, seconds):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * len(JOINTS)
        t = seconds / self.speed
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info(
            f'{label}  ->  ' + '  '.join(f'{n}={p:+.2f}' for n, p in zip(JOINTS, positions)))

        fut = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        if code != 0:
            self.get_logger().warn(f'  trajectory finished with error_code={code}')
        return True

    def send_path(self, label, points):
        """Send one trajectory made of many timed waypoints (smooth arcs)."""
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        for pos, t in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in pos]
            tt = t / self.speed
            pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1.0) * 1e9))
            traj.points.append(pt)
        traj.points[-1].velocities = [0.0] * len(JOINTS)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info(
            f'{label}  ->  {len(points)} waypoints over '
            f'{points[-1][1] / self.speed:.0f} s')
        fut = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        if code != 0:
            self.get_logger().warn(f'  trajectory finished with error_code={code}')
        return True

    def receive_package(self, slide):
        """Infeed: a package arrives into the carriage channel."""
        try:
            from spawn_payload import spawn_box, carriage_pose
        except ImportError:
            import importlib.util, os
            f = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'spawn_payload.py')
            spec = importlib.util.spec_from_file_location('spawn_payload', f)
            m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
            spawn_box, carriage_pose = m.spawn_box, m.carriage_pose
        self._n_pkg = getattr(self, '_n_pkg', 0) + 1
        x, y, z = carriage_pose(slide)
        ok, err = spawn_box(f'pkg_in_{self._n_pkg}', x, y, z)
        self.get_logger().info(
            f'   {"received" if ok else "FAILED to receive"} pkg_in_{self._n_pkg} '
            f'at ({x:.2f}, {y:.2f}, {z:.2f})')
        if not ok:
            self.get_logger().error(err)

    def run(self):
        delay = float(self.get_parameter('start_delay').value)
        if delay > 0:
            self.get_logger().info(
                f'holding still {delay:.0f} s while the shelf is loaded ...')
            end = time.time() + delay
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        # one-off: bring the carrier from its parked pose onto the conveyor-side
        # run.  After this it never leaves the loop.
        self.send('carrier -> receive position  ', [0.0, 0.0, 0.0, SHELF_TRANSFER], 5.0)
        n = 0
        while rclpy.ok() and (self.loops == 0 or n < self.loops):
            slot = FREE_SLOTS[n % len(FREE_SLOTS)]
            n += 1
            self.get_logger().info(f'===== work cycle {n}:  comptoir slot {slot + 1} =====')
            for label, *pos, secs in build_cycle(SLOT_SLIDE[slot]):
                if not rclpy.ok():
                    return
                if label.startswith('RECEIVE'):
                    self.receive_package(INFEED_SLIDE)
                if not self.send(label, pos, secs):
                    return
            # the carrier circulates the whole chain loop back to the infeed
            if not rclpy.ok() or not self.send_path(
                    'PATERNOSTER - up, over the top, down the far side, under  ',
                    paternoster_loop(SLOT_SLIDE[slot])):
                return
        self.get_logger().info('done')


def main():
    rclpy.init()
    try:
        CycleDemo().run()
    except (KeyboardInterrupt, SystemExit) as e:
        if str(e):
            print(str(e), file=sys.stderr)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
