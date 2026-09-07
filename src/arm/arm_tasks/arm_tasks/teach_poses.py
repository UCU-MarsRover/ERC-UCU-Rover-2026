#!/usr/bin/env python3
"""
teach_poses.py — save named arm poses and return to them SAFELY.

Safety model:
  * Motion goes through the normal FollowJointTrajectory action, so every
    protection in the stack still applies (0.3 rad/s hardware rate limiter,
    path tolerance abort, stiffness ramp).
  * Joints move SEQUENTIALLY, one at a time, in a gravity-aware order:
      going anywhere: wrists first, then ELBOW (fold), then shoulder, then base.
    This enforces the golden rule: the shoulder only ever moves while the
    elbow is folded / forearm load is minimal.
  * Speed is computed from --speed (default 0.15 rad/s per joint, well under
    the hardware limiter), with generous goal tolerances for non-backdrivable
    gearboxes.

Three ways to define a pose:
  1. Physically: with the real stack running, drive the arm there, then `save`.
  2. In visualization: run `gui_only:=true` (no hardware!), set the sliders,
     then `save` — it reads /joint_states, which the sliders publish.
  3. Numerically: `set` with explicit joint=value pairs, no GUI at all.

Usage:
  python3 teach_poses.py save home                    # from /joint_states (real arm OR gui sliders)
  python3 teach_poses.py set park arm_base_shoulder_joint=-2.9 arm_shoulder_forearm_joint=-3.0
                                                      # unspecified joints: kept from current
                                                      # /joint_states if available, else 0.0
  python3 teach_poses.py preview park                 # show a saved pose in RViz (no hardware!)
  python3 teach_poses.py list
  python3 teach_poses.py goto park                    # staged, slow, gravity-aware
  python3 teach_poses.py goto park --speed 0.1

Poses are stored in poses.json next to this script.

IMPORTANT for gui/set-defined poses: unlike `save` from the real arm, nobody
has proven the arm can actually stand there. Before the first `goto` to such
a pose, `preview` it and sanity-check: no self-collision, elbow-folded-ish,
within joint limits.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def _resolve_poses_file():
    """Find poses.json the same way keyboard_servo_node does: installed
    share copy, then the /opt/ws source tree, then relative to this script.
    Falls back to the last (source-tree) candidate if none exist yet.
    """
    candidates = [
        Path("/opt/ws/src/arm/arm_teleop/poses.json"),
        # arm_teleop is a sibling package of arm_tasks under src/arm/ — this
        # script itself did not move, only poses.json did (to arm_teleop,
        # alongside keyboard_servo_node.py, see arm_teleop/package.xml).
        Path(__file__).resolve().parents[2] / "arm_teleop" / "poses.json",
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, Path(get_package_share_directory("arm_teleop")) / "poses.json")
    except Exception:
        pass
    for path in candidates:
        if path.is_file():
            return path
    return candidates[-1]


POSES_FILE = _resolve_poses_file()
ACTION = "indomitus_arm_controller/follow_joint_trajectory"
JTC_CONTROLLER_NAME = "indomitus_arm_controller"
FORWARD_CONTROLLER_NAME = "indomitus_arm_forward_position_controller"

# Default motion order: least loaded / least consequential first,
# elbow folds BEFORE the shoulder moves, base last (pure yaw, gravity-neutral),
# so the arm is always as compact as possible when heavy joints move.
DEFAULT_ORDER = [
    "arm_wrist_2_end_effector_joint",
    "arm_wrist_1_wrist_2_joint",
    "arm_forearm_wrist_1_joint",
    "arm_shoulder_forearm_joint",   # elbow: fold first
    "arm_base_shoulder_joint",      # shoulder: moves with folded elbow
    "arm_mount_base_joint",         # base yaw last
]

GOAL_TOL_RAD = 0.08          # generous: sticky gearboxes park nearby
MIN_MOVE_RAD = 0.02          # skip joints already at target


class Teach(Node):
    def __init__(self):
        super().__init__("teach_poses")
        self._js = None
        self.create_subscription(JointState, "joint_states", self._on_js, 10)

    def _on_js(self, msg):
        self._js = msg

    def ensure_trajectory_controller(self):
        """Make sure JTC owns the joints before sending any goal.

        Teleop may have left the streaming controller active, in which case
        JTC's action server isn't even advertised and `goto` would fail with
        a confusing "action server not available" instead of just switching.
        """
        list_client = self.create_client(ListControllers, "controller_manager/list_controllers")
        states = {}
        if list_client.wait_for_service(timeout_sec=3.0):
            fut = list_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            if fut.result() is not None:
                states = {c.name: c.state for c in fut.result().controller}

        # STRICT errors on a controller already in its requested state —
        # if JTC is already active (the common case: teleop was never run),
        # there is nothing to switch.
        if states and states.get(JTC_CONTROLLER_NAME) == "active":
            return

        client = self.create_client(SwitchController, "controller_manager/switch_controller")
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("switch_controller unavailable — proceeding without switching")
            return
        req = SwitchController.Request()
        req.activate_controllers = [JTC_CONTROLLER_NAME]
        req.deactivate_controllers = [FORWARD_CONTROLLER_NAME]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.ok:
            self.get_logger().warn("Could not confirm JTC is active — goal may be rejected")

    def current_pose(self, timeout=3.0):
        t0 = time.monotonic()
        while self._js is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                self.get_logger().fatal("No /joint_states — is the stack running?")
                sys.exit(1)
        return dict(zip(self._js.name, self._js.position))

    def move_joint(self, client, joint, target, speed):
        pose = self.current_pose()
        if joint not in pose:
            self.get_logger().warn(f"{joint}: not in /joint_states, skipping")
            return True
        delta = target - pose[joint]
        if abs(delta) < MIN_MOVE_RAD:
            self.get_logger().info(f"{joint}: already at target ({pose[joint]:+.3f})")
            return True
        secs = max(2.0, abs(delta) / max(speed, 0.01))
        self.get_logger().info(
            f"{joint}: {pose[joint]:+.3f} -> {target:+.3f} rad "
            f"(d={delta:+.3f}) over {secs:.1f} s")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [joint]
        pt = JointTrajectoryPoint()
        pt.positions = [float(target)]
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        from control_msgs.msg import JointTolerance
        goal.goal_tolerance = [JointTolerance(name=joint, position=GOAL_TOL_RAD)]
        goal.goal_time_tolerance = Duration(sec=10)

        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f"{joint}: goal rejected")
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        res = rfut.result().result
        if res.error_code == 0:
            self.get_logger().info(f"{joint}: done")
            return True
        self.get_logger().error(
            f"{joint}: ABORTED ({res.error_string}). Stopping the sequence — "
            f"arm is holding position. Check the joint, then rerun.")
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p_save = sub.add_parser("save");  p_save.add_argument("name")
    p_set  = sub.add_parser("set")
    p_set.add_argument("name")
    p_set.add_argument("pairs", nargs="+", help="joint_name=radians ...")
    p_prev = sub.add_parser("preview"); p_prev.add_argument("name")
    p_goto = sub.add_parser("goto");  p_goto.add_argument("name")
    p_goto.add_argument("--speed", type=float, default=0.15, help="rad/s per joint")
    p_goto.add_argument("--order", default=None,
                        help="comma-separated joint names; default is the "
                             "gravity-aware order (wrists, elbow, shoulder, base)")
    sub.add_parser("list")
    # Strip a trailing --ros-args ... block (e.g. -r __ns:=/arm) before this
    # parser sees argv — otherwise it reads as unrecognized arguments and
    # exits before rclpy.init() ever gets a chance to apply the remap.
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    poses = json.loads(POSES_FILE.read_text()) if POSES_FILE.exists() else {}

    if args.cmd == "list":
        for name, p in poses.items():
            vals = "  ".join(f"{j.split('_')[1][:4]}:{v:+.2f}" for j, v in p.items())
            print(f"{name:15s} {vals}")
        return

    rclpy.init()
    node = Teach()

    if args.cmd == "save":
        pose = node.current_pose()
        poses[args.name] = pose
        POSES_FILE.write_text(json.dumps(poses, indent=2))
        print(f"[✓] saved '{args.name}':")
        for j, v in pose.items():
            print(f"      {j:38s} {v:+.4f} rad")
        rclpy.shutdown()
        return

    if args.cmd == "set":
        base = {}
        try:  # start from current /joint_states if a publisher exists (arm or sliders)
            base = node.current_pose(timeout=1.5)
        except SystemExit:
            print("[i] no /joint_states publisher; unspecified joints default to 0.0")
            base = {j: 0.0 for j in DEFAULT_ORDER}
        for pair in args.pairs:
            j, _, v = pair.partition("=")
            if j not in DEFAULT_ORDER:
                print(f"[!] unknown joint '{j}'; known: {DEFAULT_ORDER}")
                sys.exit(1)
            base[j] = float(v)
        poses[args.name] = {j: base.get(j, 0.0) for j in DEFAULT_ORDER}
        POSES_FILE.write_text(json.dumps(poses, indent=2))
        print(f"[\u2713] set '{args.name}':")
        for j, v in poses[args.name].items():
            print(f"      {j:38s} {v:+.4f} rad")
        rclpy.shutdown()
        return

    if args.cmd == "preview":
        if args.name not in poses:
            print(f"pose '{args.name}' not found; available: {list(poses)}")
            sys.exit(1)
        pub = node.create_publisher(JointState, "joint_states", 10)
        print(f"[i] publishing '{args.name}' to /joint_states at 10 Hz. "
              "Run ONLY robot_state_publisher + RViz alongside (no sliders, "
              "no real stack!). Ctrl+C to stop.")
        try:
            while rclpy.ok():
                msg = JointState()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.name = list(poses[args.name].keys())
                msg.position = [float(v) for v in poses[args.name].values()]
                pub.publish(msg)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        rclpy.shutdown()
        return

    # goto
    if args.name not in poses:
        print(f"pose '{args.name}' not found; available: {list(poses)}")
        sys.exit(1)
    target = poses[args.name]
    order = args.order.split(",") if args.order else DEFAULT_ORDER

    node.ensure_trajectory_controller()

    client = ActionClient(node, FollowJointTrajectory, ACTION)
    if not client.wait_for_server(timeout_sec=3.0):
        print("action server not available — is the stack running?")
        sys.exit(1)

    print(f"[i] going to '{args.name}' sequentially: wrists -> elbow -> shoulder -> base")
    for joint in order:
        if joint not in target:
            continue
        ok = node.move_joint(client, joint, target[joint], args.speed)
        if not ok:
            sys.exit(1)
    print(f"[✓] arrived at '{args.name}'")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
