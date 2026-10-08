#!/usr/bin/env python3
"""Week 5: record odometry trials and summarise their error.

record   capture /odom before and after each run, ask for the ruler
         measurement, and append one row per trial to a CSV file
summary  print the measurement table plus mean / standard deviation of
         the error, for every (method, mode) group in the CSV file(s)
"""

import argparse
import csv
import math
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

CSV_FIELDS = [
    "method",
    "mode",
    "trial",
    "target_m",
    "actual_m",
    "odom_m",
    "odom_forward_m",
    "odom_lateral_m",
    "odom_dyaw_deg",
    "start_x",
    "start_y",
    "start_yaw_deg",
    "end_x",
    "end_y",
    "end_yaw_deg",
    "time",
]


class QuitRequested(Exception):
    pass


# ---------------------------------------------------------------- geometry


def yaw_from_quaternion(q) -> float:
    """Heading (radians) of a geometry_msgs/Quaternion about the z axis."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_degrees(angle: float) -> float:
    """Wrap an angle in degrees into [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def pose_of(odom):
    """(x, y, yaw radians) from a nav_msgs/Odometry message."""
    pose = odom.pose.pose
    return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)


def odom_delta(start, end):
    """
    Motion between two (x, y, yaw) poses.

    distance  straight-line distance between the two positions; this is
              what a ruler laid between the start and end marks measures
    forward   the part of that motion along the starting heading
    lateral   the part sideways to the starting heading (+ = left)
    dyaw      change in heading, degrees
    """
    x0, y0, yaw0 = start
    x1, y1, yaw1 = end
    dx, dy = x1 - x0, y1 - y0
    return {
        "distance": math.hypot(dx, dy),
        "forward": dx * math.cos(yaw0) + dy * math.sin(yaw0),
        "lateral": -dx * math.sin(yaw0) + dy * math.cos(yaw0),
        "dyaw": wrap_degrees(math.degrees(yaw1 - yaw0)),
    }


# ---------------------------------------------------------------- ROS side


class OdomRecorder:
    """Small rclpy wrapper: grab one fresh /odom message, run DriveDistance."""

    def __init__(self, topic: str):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self._rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("odom_lab")
        self._latest = None
        # Sensor-data QoS (best effort) matches both reliable and
        # best-effort publishers, so it works whatever the robot uses.
        self.node.create_subscription(
            Odometry, topic, self._on_odom, qos_profile_sensor_data
        )
        self._drive_client = None

    def _on_odom(self, msg):
        self._latest = msg

    def capture(self, timeout: float = 5.0):
        """Wait for a message published *after* this call, and return it."""
        self._latest = None
        deadline = time.monotonic() + timeout
        while self._latest is None:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "No /odom message received. Did you source "
                    "robot_profiles/select_robot.bash for your robot?"
                )
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._latest

    def drive_distance(self, distance: float, speed: float) -> None:
        """Send one irobot_create_msgs DriveDistance goal and wait for it."""
        from irobot_create_msgs.action import DriveDistance
        from rclpy.action import ActionClient

        if self._drive_client is None:
            self._drive_client = ActionClient(self.node, DriveDistance, "drive_distance")
        if not self._drive_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/drive_distance action server not available.")

        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.max_translation_speed = speed
        send_future = self._drive_client.send_goal_async(goal)
        self._rclpy.spin_until_future_complete(self.node, send_future)
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("DriveDistance goal was rejected.")

        result_future = handle.get_result_async()
        self._rclpy.spin_until_future_complete(self.node, result_future)
        status = result_future.result().status
        if status != 4:  # action_msgs/msg/GoalStatus.STATUS_SUCCEEDED
            raise RuntimeError(f"DriveDistance ended with Action status {status}.")

    def shutdown(self):
        self.node.destroy_node()
        self._rclpy.shutdown()


def save_odom_yaml(msg, path: Path) -> None:
    from rosidl_runtime_py import message_to_yaml

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(message_to_yaml(msg))


# ---------------------------------------------------------------- record


def ask(prompt: str) -> str:
    answer = input(prompt).strip()
    if answer.lower() in ("q", "quit", "exit"):
        raise QuitRequested
    return answer


def ask_metres(prompt: str) -> float:
    while True:
        answer = ask(prompt)
        try:
            return float(answer)
        except ValueError:
            print("  Enter a number in metres (e.g. 0.987), or q to quit.")


def next_trial_number(csv_path: Path, method: str, mode: str) -> int:
    if not csv_path.exists():
        return 1
    with csv_path.open(newline="") as f:
        trials = [
            int(row["trial"])
            for row in csv.DictReader(f)
            if row["method"] == method and row["mode"] == mode
        ]
    return max(trials, default=0) + 1


def append_row(csv_path: Path, row: dict) -> None:
    new_file = not csv_path.exists()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new_file:
            writer.writeheader()
        writer.writerow(row)


def print_pose(label: str, pose) -> None:
    x, y, yaw = pose
    print(f"  {label}: x={x:+.4f} m  y={y:+.4f} m  yaw={math.degrees(yaw):+.2f} deg")


def record(args) -> None:
    if args.drive_distance and args.mode != "line":
        sys.exit("--drive-distance only applies to --mode line.")

    target = 0.0 if args.mode == "circle" else args.target
    csv_path = Path(args.csv)
    yaml_dir = csv_path.parent / (csv_path.stem + "_odom")
    first = next_trial_number(csv_path, args.method, args.mode)

    if args.mode == "circle":
        drive_hint = "drive the circle now (e.g. `arc left 0.5 360` in the hw4 executive)"
        ruler_prompt = "  Ruler: distance from start mark to end position (m): "
    elif args.drive_distance:
        drive_hint = None
        ruler_prompt = "  Ruler: distance actually travelled (m): "
    else:
        drive_hint = f"drive {target:g} m now (e.g. `move {target:g}` in the hw4 executive)"
        ruler_prompt = "  Ruler: distance actually travelled (m): "

    print(f"Recording {args.trials} '{args.method}' {args.mode} trials into {csv_path}")
    print("Type q at any prompt to stop.\n")

    recorder = OdomRecorder(args.topic)
    try:
        for trial in range(first, first + args.trials):
            print(f"--- Trial {trial}")
            ask("  Mark the start position, then press Enter to capture start odometry. ")
            start_msg = recorder.capture()
            start = pose_of(start_msg)
            print_pose("start", start)

            if drive_hint is None:
                print(f"  Sending DriveDistance {target:g} m at {args.speed:g} m/s...")
                recorder.drive_distance(target, args.speed)
                time.sleep(0.5)  # let the robot settle before reading /odom
            else:
                ask(f"  In the other terminal, {drive_hint}.\n  Press Enter once it has stopped. ")

            end_msg = recorder.capture()
            end = pose_of(end_msg)
            print_pose("end  ", end)
            delta = odom_delta(start, end)
            print(
                f"  odometry: moved {delta['distance']:.4f} m "
                f"(forward {delta['forward']:+.4f}, lateral {delta['lateral']:+.4f}), "
                f"turned {delta['dyaw']:+.2f} deg"
            )

            save_odom_yaml(start_msg, yaml_dir / f"{args.method}_{args.mode}_{trial:02d}_start.yaml")
            save_odom_yaml(end_msg, yaml_dir / f"{args.method}_{args.mode}_{trial:02d}_end.yaml")

            actual = ask_metres(ruler_prompt)
            append_row(
                csv_path,
                {
                    "method": args.method,
                    "mode": args.mode,
                    "trial": trial,
                    "target_m": target,
                    "actual_m": actual,
                    "odom_m": round(delta["distance"], 5),
                    "odom_forward_m": round(delta["forward"], 5),
                    "odom_lateral_m": round(delta["lateral"], 5),
                    "odom_dyaw_deg": round(delta["dyaw"], 3),
                    "start_x": round(start[0], 5),
                    "start_y": round(start[1], 5),
                    "start_yaw_deg": round(math.degrees(start[2]), 3),
                    "end_x": round(end[0], 5),
                    "end_y": round(end[1], 5),
                    "end_yaw_deg": round(math.degrees(end[2]), 3),
                    "time": datetime.now().isoformat(timespec="seconds"),
                },
            )
            print(
                f"  saved: actual error {actual - target:+.4f} m, "
                f"odometry error {delta['distance'] - target:+.4f} m\n"
            )
    except (QuitRequested, EOFError, KeyboardInterrupt):
        print("\nStopped.")
    finally:
        recorder.shutdown()

    if csv_path.exists():
        print("Summary so far:\n")
        summarise([csv_path])


# ---------------------------------------------------------------- summary


def stats(values):
    """(mean, sample standard deviation) — stdev needs at least 2 values."""
    if not values:
        return float("nan"), float("nan")
    mean = statistics.mean(values)
    stdev = statistics.stdev(values) if len(values) > 1 else float("nan")
    return mean, stdev


def summarise(csv_paths) -> None:
    groups = {}
    for path in csv_paths:
        with Path(path).open(newline="") as f:
            for row in csv.DictReader(f):
                groups.setdefault((row["method"], row["mode"]), []).append(row)

    if not groups:
        print("No trials recorded yet.")
        return

    for (method, mode), rows in groups.items():
        rows.sort(key=lambda r: int(r["trial"]))
        target = float(rows[0]["target_m"])
        what = "distance from start" if mode == "circle" else "distance travelled"
        print(f"### {method} — {mode} (target {target:g} m, {what})\n")
        print("| Trial | Actual (m) | Actual error (m) | Odometry (m) | Odometry error (m) | Odom heading change (deg) |")
        print("|---:|---:|---:|---:|---:|---:|")
        actual_errors, odom_errors = [], []
        for r in rows:
            actual, odom = float(r["actual_m"]), float(r["odom_m"])
            actual_errors.append(actual - target)
            odom_errors.append(odom - target)
            print(
                f"| {r['trial']} | {actual:.4f} | {actual - target:+.4f} | "
                f"{odom:.4f} | {odom - target:+.4f} | {float(r['odom_dyaw_deg']):+.2f} |"
            )
        a_mean, a_std = stats(actual_errors)
        o_mean, o_std = stats(odom_errors)
        print()
        print("| Error | Mean (m) | Std dev (m) |")
        print("|---|---:|---:|")
        print(f"| Actual (ruler) | {a_mean:+.4f} | {a_std:.4f} |")
        print(f"| Odometry | {o_mean:+.4f} | {o_std:.4f} |")
        print(f"\nn = {len(rows)}; std dev is the sample standard deviation (n - 1).\n")


# ---------------------------------------------------------------- CLI


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    rec = sub.add_parser("record", help="record trials")
    rec.add_argument("--method", required=True, help="label for the CSV, e.g. my_action or drive_distance")
    rec.add_argument("--mode", choices=["line", "circle"], default="line")
    rec.add_argument("--target", type=float, default=1.0, help="line distance in metres (default 1.0)")
    rec.add_argument("--trials", type=int, default=10)
    rec.add_argument("--csv", default="data/trials.csv")
    rec.add_argument("--topic", default="/odom")
    rec.add_argument("--drive-distance", action="store_true", help="drive with the built-in DriveDistance action")
    rec.add_argument("--speed", type=float, default=0.10, help="DriveDistance max speed, m/s (default matches hw4)")

    summ = sub.add_parser("summary", help="print tables and statistics")
    summ.add_argument("csv", nargs="*", default=["data/trials.csv"])

    args = parser.parse_args()
    if args.command == "record":
        record(args)
    else:
        summarise(args.csv)


if __name__ == "__main__":
    main()
