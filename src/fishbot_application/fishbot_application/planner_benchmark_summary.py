import argparse
import csv
import math
import os
from collections import defaultdict
from typing import Dict, List, Optional


SUMMARY_HEADER = [
    "planner_name",
    "samples",
    "success_count",
    "success_rate_pct",
    "stuck_count",
    "stuck_rate_pct",
    "avg_time_all_s",
    "avg_time_success_s",
    "avg_recovery_count",
    "avg_wall_hugging_level",
    "wall_hugging_samples",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize planner benchmark CSV results."
    )
    parser.add_argument(
        "--input-csv",
        required=True,
        help="Path to planner benchmark result CSV",
    )
    parser.add_argument(
        "--output-csv",
        default="",
        help="Optional path to save summarized CSV",
    )
    parser.add_argument(
        "--output-md",
        default="",
        help="Optional path to save markdown table",
    )
    parser.add_argument(
        "--sort-by",
        choices=["success", "time", "stuck", "planner"],
        default="success",
        help="Sort metric for output table",
    )
    return parser.parse_args()


def to_float(value: str) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if text == "":
        return None
    try:
        return float(text)
    except ValueError:
        return None


def to_int(value: str) -> Optional[int]:
    v = to_float(value)
    if v is None:
        return None
    return int(v)


def safe_mean(values: List[float]) -> Optional[float]:
    if not values:
        return None
    return sum(values) / len(values)


def format_num(value: Optional[float], ndigits: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value:.{ndigits}f}"


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)


def read_rows(input_csv: str) -> List[Dict]:
    with open(input_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"No data rows in {input_csv}")
    return rows


def summarize(rows: List[Dict]) -> List[Dict]:
    # 按 planner_name 分组，统计成功率、卡死率、平均耗时等聚合指标。
    grouped = defaultdict(list)
    for row in rows:
        planner = str(row.get("planner_name", "")).strip()
        if planner == "":
            planner = "UNKNOWN"
        grouped[planner].append(row)

    summary_rows = []
    for planner, items in grouped.items():
        success_vals = [to_int(r.get("success", "")) for r in items]
        stuck_vals = [to_int(r.get("stuck_or_not", "")) for r in items]
        time_vals = [to_float(r.get("time_to_goal_s", "")) for r in items]
        recovery_vals = [to_float(r.get("recovery_count", "")) for r in items]
        wall_vals = [to_float(r.get("wall_hugging_level", "")) for r in items]

        valid_success = [v for v in success_vals if v is not None]
        valid_stuck = [v for v in stuck_vals if v is not None]
        valid_time = [v for v in time_vals if v is not None]
        valid_recovery = [v for v in recovery_vals if v is not None]
        valid_wall = [v for v in wall_vals if v is not None]

        success_count = sum(1 for v in valid_success if v == 1)
        stuck_count = sum(1 for v in valid_stuck if v == 1)
        success_rate = (
            100.0 * success_count / len(valid_success) if valid_success else None
        )
        stuck_rate = 100.0 * stuck_count / len(valid_stuck) if valid_stuck else None

        success_times = [
            t for (t, s) in zip(time_vals, success_vals) if t is not None and s == 1
        ]

        summary_rows.append(
            {
                "planner_name": planner,
                "samples": len(items),
                "success_count": success_count,
                "success_rate_pct": success_rate,
                "stuck_count": stuck_count,
                "stuck_rate_pct": stuck_rate,
                "avg_time_all_s": safe_mean(valid_time),
                "avg_time_success_s": safe_mean(success_times),
                "avg_recovery_count": safe_mean(valid_recovery),
                "avg_wall_hugging_level": safe_mean(valid_wall),
                "wall_hugging_samples": len(valid_wall),
            }
        )
    return summary_rows


def sort_summary(rows: List[Dict], sort_by: str) -> List[Dict]:
    if sort_by == "planner":
        return sorted(rows, key=lambda r: r["planner_name"])
    if sort_by == "time":
        return sorted(
            rows,
            key=lambda r: (
                r["avg_time_success_s"] is None,
                r["avg_time_success_s"] if r["avg_time_success_s"] is not None else 1e18,
            ),
        )
    if sort_by == "stuck":
        return sorted(
            rows,
            key=lambda r: (
                r["stuck_rate_pct"] is None,
                r["stuck_rate_pct"] if r["stuck_rate_pct"] is not None else 1e18,
            ),
        )
    # 默认按成功率排序（同成功率时再比较成功样本平均耗时）。
    return sorted(
        rows,
        key=lambda r: (
            r["success_rate_pct"] is None,
            -(r["success_rate_pct"] if r["success_rate_pct"] is not None else -1e18),
            r["avg_time_success_s"] if r["avg_time_success_s"] is not None else 1e18,
        ),
    )


def write_summary_csv(path: str, rows: List[Dict]) -> None:
    ensure_parent(path)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_HEADER)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "planner_name": row["planner_name"],
                    "samples": row["samples"],
                    "success_count": row["success_count"],
                    "success_rate_pct": format_num(row["success_rate_pct"], 2),
                    "stuck_count": row["stuck_count"],
                    "stuck_rate_pct": format_num(row["stuck_rate_pct"], 2),
                    "avg_time_all_s": format_num(row["avg_time_all_s"], 3),
                    "avg_time_success_s": format_num(row["avg_time_success_s"], 3),
                    "avg_recovery_count": format_num(row["avg_recovery_count"], 3),
                    "avg_wall_hugging_level": format_num(row["avg_wall_hugging_level"], 3),
                    "wall_hugging_samples": row["wall_hugging_samples"],
                }
            )


def build_markdown_table(rows: List[Dict]) -> str:
    lines = []
    lines.append(
        "| Planner | Samples | Success | Success Rate(%) | Stuck | Stuck Rate(%) | Avg Time Success(s) | Avg Recovery | Avg Wall Hugging |"
    )
    lines.append(
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"
    )
    for row in rows:
        lines.append(
            "| "
            + f"{row['planner_name']} | "
            + f"{row['samples']} | "
            + f"{row['success_count']} | "
            + f"{format_num(row['success_rate_pct'], 2)} | "
            + f"{row['stuck_count']} | "
            + f"{format_num(row['stuck_rate_pct'], 2)} | "
            + f"{format_num(row['avg_time_success_s'], 3)} | "
            + f"{format_num(row['avg_recovery_count'], 3)} | "
            + f"{format_num(row['avg_wall_hugging_level'], 3)}"
            + " |"
        )
    return "\n".join(lines)


def print_console(rows: List[Dict]) -> None:
    print(build_markdown_table(rows))


def main():
    args = parse_args()
    rows = read_rows(args.input_csv)
    summary_rows = summarize(rows)
    summary_rows = sort_summary(summary_rows, args.sort_by)

    print_console(summary_rows)

    if args.output_csv:
        write_summary_csv(args.output_csv, summary_rows)
        print(f"\nSaved summary CSV: {args.output_csv}")

    if args.output_md:
        ensure_parent(args.output_md)
        with open(args.output_md, "w", encoding="utf-8") as f:
            f.write(build_markdown_table(summary_rows))
            f.write("\n")
        print(f"Saved summary Markdown: {args.output_md}")


if __name__ == "__main__":
    main()
