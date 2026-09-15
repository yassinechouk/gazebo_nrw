#!/usr/bin/env python3
"""
Spawn grey packages into the running Gazebo world.

All packages are spawned concurrently, so a pre-loaded shelf appears complete
in one instant rather than filling slot by slot.

Two stations, matching how the cell actually works:

  carriage  one package dropped into the green carriage channel, sized to fit
            between its side walls.  This is the package the pusher transfers.
  shelf     packages pre-loaded into the red comptoir's slots, so the shelf
            arrives already carrying stock.

    ros2 run nrw_cell_sim spawn_payload.py                       # both
    ros2 run nrw_cell_sim spawn_payload.py --ros-args -p where:=shelf -p count:=6
    ros2 run nrw_cell_sim spawn_payload.py --ros-args -p where:=carriage -p slide:=-1.85
"""
import concurrent.futures
import subprocess
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WORLD = 'nrw_cell_world'
GROUND = 3.592          # CAD base frame -> world Z

# ---- cell geometry, base frame, every axis at zero -------------------------
CARRIAGE_X0 = -2.163    # channel centre in X when carriage_slide = 0
CARRIAGE_Y = -5.723
CARRIAGE_DECK_Z = -1.470          # floor the package rests on
CHANNEL_WIDTH = 0.384             # clear width between the side walls

SHELF_Y = -3.818
SHELF_DECK_Z = -1.553
# Centre of each comptoir slot, measured off the mesh: 11 dividers 30 mm wide on
# a 320 mm pitch, leaving twelve 290 mm slots.
SHELF_SLOTS = [-3.193, -2.873, -2.553, -2.233, -1.913, -1.593, -1.273, -0.953, -0.633, -0.313, 0.007, 0.324]

# a package must clear the 0.290 m slot as well as the 0.384 m channel
PKG = (0.20, 0.30, 0.12)          # X, Y, Z
PKG_MASS = 4.0

SDF = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{m}</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface>
          <friction><ode><mu>0.85</mu><mu2>0.85</mu2></ode></friction>
          <contact><ode><kp>5e5</kp><kd>2e3</kd><max_vel>0.05</max_vel></ode></contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>0.42 0.44 0.46 1</ambient>
          <diffuse>0.62 0.64 0.66 1</diffuse>
          <specular>0.18 0.18 0.18 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def spawn_box(name, x, y, z, size=PKG, mass=PKG_MASS, timeout=60):
    sx, sy, sz = size
    k = mass / 12.0
    sdf = SDF.format(name=name, m=mass, sx=sx, sy=sy, sz=sz,
                     ixx=k * (sy**2 + sz**2), iyy=k * (sx**2 + sz**2),
                     izz=k * (sx**2 + sy**2))
    cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
           '-world', WORLD, '-string', sdf, '-name', name,
           '-allow_renaming', 'true',
           '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', f'{z:.4f}']
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.returncode == 0, r.stderr.strip()[-300:]


def carriage_pose(slide=0.0, size=PKG):
    """World pose of a package sitting in the carriage channel."""
    return (CARRIAGE_X0 + slide, CARRIAGE_Y,
            CARRIAGE_DECK_Z + size[2] / 2.0 + 0.01 + GROUND)


def shelf_pose(slot_index, shelf_extend=0.0, lift=0.0, size=PKG):
    """World pose of a package sitting in comptoir slot `slot_index`."""
    return (SHELF_SLOTS[slot_index % len(SHELF_SLOTS)],
            SHELF_Y - shelf_extend,
            SHELF_DECK_Z + size[2] / 2.0 + 0.01 + GROUND + lift)


class Spawner(Node):
    def __init__(self):
        super().__init__('nrw_cell_spawn_payload')
        self._joints = {}
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.declare_parameter('where', 'both')      # carriage | shelf | both
        self.declare_parameter('count', 5)           # packages on the shelf
        self.declare_parameter('slide', 0.0)         # carriage_slide at spawn time
        self.declare_parameter('delay', 3.0)
        self.declare_parameter('prefix', 'pkg')

    def _on_joints(self, msg):
        self._joints.update(dict(zip(msg.name, msg.position)))

    def wait_for_joints(self, timeout=5.0):
        """Read the live axis positions, so packages land where the machine
        actually is rather than where it was at t=0."""
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end and len(self._joints) < 4:
            rclpy.spin_once(self, timeout_sec=0.1)
        if len(self._joints) < 4:
            self.get_logger().warn(
                '/joint_states not seen; assuming every axis is at zero')
        return self._joints

    def run(self):
        where = self.get_parameter('where').value
        n = int(self.get_parameter('count').value)
        slide = float(self.get_parameter('slide').value)
        delay = float(self.get_parameter('delay').value)
        prefix = self.get_parameter('prefix').value
        if delay > 0:
            time.sleep(delay)

        # Build the whole batch first, then fire every spawn at once, so the
        # pre-loaded packages all appear in the same instant instead of one
        # every half second.
        q = self.wait_for_joints()
        q_shelf = q.get('shelf_extend', 0.0)
        q_lift = q.get('lift', 0.0)
        if 'carriage_slide' in q:
            slide = q['carriage_slide']

        jobs = []
        if where in ('shelf', 'both'):
            # leave the right-hand slots free so transferred packages have
            # somewhere to land
            for i in range(min(n, len(SHELF_SLOTS) - 3)):
                x, y, z = shelf_pose(i, shelf_extend=q_shelf, lift=q_lift)
                jobs.append((f'{prefix}_shelf_{i+1}', x, y, z,
                             f'comptoir slot {i + 1}'))
        if where in ('carriage', 'both'):
            x, y, z = carriage_pose(slide)
            jobs.append((f'{prefix}_carriage', x, y, z, 'the carriage channel'))

        with concurrent.futures.ThreadPoolExecutor(max_workers=len(jobs)) as pool:
            futures = {pool.submit(spawn_box, nm, x, y, z): (nm, x, y, z, lbl)
                       for nm, x, y, z, lbl in jobs}
            results = {}
            for fut in concurrent.futures.as_completed(futures):
                nm, x, y, z, lbl = futures[fut]
                try:
                    results[nm] = (fut.result(), x, y, z, lbl)
                except Exception as exc:                      # noqa: BLE001
                    results[nm] = ((False, str(exc)), x, y, z, lbl)

        for nm, _x, _y, _z, _lbl in jobs:
            (ok, err), x, y, z, lbl = results[nm]
            self.get_logger().info(
                f'{"loaded " if ok else "FAILED "} {nm} into {lbl} '
                f'at ({x:.2f}, {y:.2f}, {z:.2f})')
            if not ok:
                self.get_logger().error(err)
        self.get_logger().info(
            f'{sum(1 for r in results.values() if r[0][0])}/{len(jobs)} packages '
            f'spawned together  [{PKG[0]}x{PKG[1]}x{PKG[2]} m, {PKG_MASS} kg each]')


def main():
    rclpy.init()
    try:
        Spawner().run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
