import argparse
import csv
import math
import os
import time
from typing import Dict, List, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


CSV_HEADER = [
    "planner_name",
    "run_id",
    "goal_id",
    "success",
    "time_to_goal_s",
    "stuck_or_not",
    "wall_hugging_level",
    "recovery_count",
    "note",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Planner benchmark runner for Nav2. Run one planner config at a time."
    )
    parser.add_argument(
        "--goals-file",
        required=True,
        help="Path to planner_benchmark_goals.yaml",
    )
    parser.add_argument(
        "--output-csv",
        required=True,
        help="Output CSV path",
    )
    parser.add_argument(
        "--planner-name",
        required=True,
        help="Planner name written into CSV, e.g. SmacPlanner2D/NavfnPlanner/CustomPlanner",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=3,
        help="Number of repeated runs per goal (default: 3)",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=180.0,
        help="Timeout per goal in seconds (default: 180)",
    )
    parser.add_argument(
        "--wall-hugging-default",
        default="",
        help="Default value written into wall_hugging_level column (manual rating can be filled later)",
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="Append to output CSV if file exists; otherwise overwrite",
    )
    return parser.parse_args()


def load_goals(goals_file: str) -> Tuple[str, List[Dict]]:
    # 从 benchmark_goals 中读取坐标系与目标点列表。
    with open(goals_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    benchmark = data.get("benchmark_goals", {})
    frame_id = benchmark.get("frame_id", "map")
    points = benchmark.get("points", [])
    if not points:
        raise ValueError(f"No goals found in {goals_file}")
    return frame_id, points


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    # 二维平面仅绕 Z 轴旋转，转换为四元数供 PoseStamped 使用。
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def build_goal_pose(
    navigator: BasicNavigator,
    frame_id: str,
    x: float,
    y: float,
    yaw_rad: float,
) -> PoseStamped:
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = frame_id
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(float(yaw_rad))
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw
    return goal_pose


def run_single_goal(
    navigator: BasicNavigator,
    goal_pose: PoseStamped,
    timeout_s: float,
) -> Dict:
    # 执行单个目标点导航，记录成功性、耗时、恢复次数与超时信息。
    start_wall_time = time.monotonic()
    max_recoveries = 0
    note = ""
    timeout_triggered = False

    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            if hasattr(feedback, "number_of_recoveries"):
                max_recoveries = max(max_recoveries, int(feedback.number_of_recoveries))

            nav_time = Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9
            if nav_time > timeout_s:
                timeout_triggered = True
                note = f"timeout>{timeout_s:.1f}s"
                navigator.cancelTask()
                break

    elapsed = time.monotonic() - start_wall_time
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        success = 1
        stuck_or_not = 0
        if not note:
            note = "ok"
    elif result == TaskResult.CANCELED:
        success = 0
        stuck_or_not = 1
        if not note:
            note = "canceled"
    elif result == TaskResult.FAILED:
        success = 0
        stuck_or_not = 1
        if not note:
            note = "failed"
    else:
        success = 0
        stuck_or_not = 1
        if not note:
            note = "invalid_result"

    if timeout_triggered and "timeout" not in note:
        note = f"timeout>{timeout_s:.1f}s"

    return {
        "success": success,
        "time_to_goal_s": round(elapsed, 3),
        "stuck_or_not": stuck_or_not,
        "recovery_count": max_recoveries,
        "note": note,
    }


def ensure_output_dir(output_csv: str) -> None:
    output_dir = os.path.dirname(os.path.abspath(output_csv))
    os.makedirs(output_dir, exist_ok=True)


def open_csv_writer(output_csv: str, append: bool):
    # append=true 且文件已存在时追加写入；否则覆盖并重写表头。
    ensure_output_dir(output_csv)
    file_exists = os.path.exists(output_csv)
    mode = "a" if append else "w"
    f = open(output_csv, mode, newline="", encoding="utf-8")
    writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
    if not append or (append and not file_exists):
        writer.writeheader()
    return f, writer


def main():
    args = parse_args()
    if args.runs < 1:
        raise ValueError("--runs must be >= 1")
    if args.timeout_s <= 0.0 or not math.isfinite(args.timeout_s):
        raise ValueError("--timeout-s must be a positive finite number")

    frame_id, goals = load_goals(args.goals_file)

    rclpy.init()
    navigator = BasicNavigator()
    navigator.get_logger().info("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    navigator.get_logger().info("Nav2 is active, benchmark starts.")

    csv_file, csv_writer = open_csv_writer(args.output_csv, args.append)
    try:
        for run_id in range(1, args.runs + 1):
            navigator.get_logger().info(f"===== Run {run_id}/{args.runs} =====")
            for goal in goals:
                goal_id = str(goal["goal_id"])
                x = float(goal["x"])
                y = float(goal["y"])
                yaw_rad = float(goal.get("yaw_rad", 0.0))
                scenario = str(goal.get("scenario", ""))

                navigator.get_logger().info(
                    f"[{args.planner_name}] run={run_id} goal={goal_id} "
                    f"scenario={scenario} x={x:.2f} y={y:.2f} yaw={yaw_rad:.2f}"
                )

                goal_pose = build_goal_pose(navigator, frame_id, x, y, yaw_rad)
                result = run_single_goal(
                    navigator=navigator,
                    goal_pose=goal_pose,
                    timeout_s=args.timeout_s,
                )

                row = {
                    "planner_name": args.planner_name,
                    "run_id": run_id,
                    "goal_id": goal_id,
                    "success": result["success"],
                    "time_to_goal_s": result["time_to_goal_s"],
                    "stuck_or_not": result["stuck_or_not"],
                    "wall_hugging_level": args.wall_hugging_default,
                    "recovery_count": result["recovery_count"],
                    "note": result["note"],
                }
                csv_writer.writerow(row)
                csv_file.flush()
                navigator.get_logger().info(
                    f"Result goal={goal_id}: success={row['success']} "
                    f"time={row['time_to_goal_s']}s recoveries={row['recovery_count']} note={row['note']}"
                )
    finally:
        csv_file.close()
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
