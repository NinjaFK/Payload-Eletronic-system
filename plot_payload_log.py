#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def to_float(value: str):
    if value is None:
        return math.nan
    text = value.strip().lower()
    if text in ("", "nan", "na", "null"):
        return math.nan
    return float(text)


def read_log(path: Path):
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"No data rows in {path}")

    ms = [to_float(r["ms"]) for r in rows]
    t0 = ms[0]
    t_s = [(m - t0) / 1000.0 for m in ms]
    return rows, t_s


def read_logs(paths):
    all_rows = []
    for path in paths:
        with path.open("r", newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        if not rows:
            continue
        all_rows.extend(rows)

    if not all_rows:
        raise ValueError("No data rows found in selected log files")

    ms = [to_float(r["ms"]) for r in all_rows]
    t0 = ms[0]
    t_s = [(m - t0) / 1000.0 for m in ms]
    return all_rows, t_s


def series(rows, key):
    return [to_float(r.get(key, "")) for r in rows]


def series_any(rows, keys):
    for key in keys:
        values = series(rows, key)
        if has_real(values):
            return values
    return [math.nan] * len(rows)


def has_real(values):
    return any(not math.isnan(v) for v in values)


def save_plot(out_dir: Path, name: str, t_s, y_map, title: str):
    available = {k: v for k, v in y_map.items() if has_real(v)}
    if not available:
        return None

    fig, ax = plt.subplots(figsize=(11, 4.5))
    for label, values in available.items():
        ax.plot(t_s, values, label=label, linewidth=1.2)
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()

    out_path = out_dir / f"{name}.png"
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return out_path


def plot_file(log_path: Path, out_dir: Path):
    rows, t_s = read_log(log_path)
    out_dir.mkdir(parents=True, exist_ok=True)

    stem = log_path.stem
    results = []

    results.append(
        save_plot(
            out_dir,
            f"{stem}_adxl",
            t_s,
            {
                "adxl_x": series(rows, "adxl_x"),
                "adxl_y": series(rows, "adxl_y"),
                "adxl_z": series(rows, "adxl_z"),
            },
            "ADXL375",
        )
    )
    results.append(
        save_plot(
            out_dir,
            f"{stem}_imu1_accel",
            t_s,
            {
                "imu1_ax": series_any(rows, ["imu1_ax_ms2", "imu1_ax"]),
                "imu1_ay": series_any(rows, ["imu1_ay_ms2", "imu1_ay"]),
                "imu1_az": series_any(rows, ["imu1_az_ms2", "imu1_az"]),
            },
            "IMU1 Accel",
        )
    )
    results.append(
        save_plot(
            out_dir,
            f"{stem}_imu1_gyro",
            t_s,
            {
                "imu1_gx": series_any(rows, ["imu1_gx_rads", "imu1_gx"]),
                "imu1_gy": series_any(rows, ["imu1_gy_rads", "imu1_gy"]),
                "imu1_gz": series_any(rows, ["imu1_gz_rads", "imu1_gz"]),
            },
            "IMU1 Gyro",
        )
    )
    results.append(
        save_plot(
            out_dir,
            f"{stem}_imu2_accel",
            t_s,
            {
                "imu2_ax": series_any(rows, ["imu2_ax_ms2", "imu2_ax"]),
                "imu2_ay": series_any(rows, ["imu2_ay_ms2", "imu2_ay"]),
                "imu2_az": series_any(rows, ["imu2_az_ms2", "imu2_az"]),
            },
            "IMU2 Accel",
        )
    )
    results.append(
        save_plot(
            out_dir,
            f"{stem}_imu2_gyro",
            t_s,
            {
                "imu2_gx": series_any(rows, ["imu2_gx_rads", "imu2_gx"]),
                "imu2_gy": series_any(rows, ["imu2_gy_rads", "imu2_gy"]),
                "imu2_gz": series_any(rows, ["imu2_gz_rads", "imu2_gz"]),
            },
            "IMU2 Gyro",
        )
    )
    results.append(
        save_plot(
            out_dir,
            f"{stem}_bmp",
            t_s,
            {
                "temp_c": series(rows, "temp_c"),
                "press_hpa": series(rows, "press_hpa"),
                "alt_m": series(rows, "alt_m"),
            },
            "BMP388",
        )
    )
    results.append(
        save_plot(
            out_dir,
            f"{stem}_flow_valve",
            t_s,
            {
                "flow_hz": series(rows, "flow_hz"),
                "flow_lpm": series(rows, "flow_lpm"),
                "valve": series(rows, "valve"),
            },
            "Flow + Valve",
        )
    )

    return [p for p in results if p is not None]


def latest_log_file(base: Path):
    candidates = sorted(base.glob("log*.csv"))
    if not candidates:
        raise FileNotFoundError("No log*.csv files found")
    return candidates[-1]


def all_log_files(base: Path):
    candidates = sorted(base.glob("log*.csv"))
    if not candidates:
        raise FileNotFoundError(f"No log*.csv files found in {base}")
    return candidates


def main():
    parser = argparse.ArgumentParser(description="Plot payload CSV log(s) by sensor.")
    parser.add_argument(
        "logfile",
        nargs="?",
        help=(
            "Path to log CSV or directory (default: all log*.csv in ./logfiles, "
            "or in current dir if ./logfiles does not exist)"
        ),
    )
    parser.add_argument(
        "--out",
        default="plots",
        help="Output directory for PNG files (default: plots)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Combine all log*.csv files from the selected directory",
    )
    args = parser.parse_args()

    cwd = Path.cwd()
    out_dir = Path(args.out)
    selected = Path(args.logfile) if args.logfile else None

    if selected is None:
        base = cwd / "logfiles"
        if not base.exists():
            base = cwd
        log_paths = all_log_files(base)
    elif selected.is_dir() or args.all:
        base = selected if selected.is_dir() else selected.parent
        log_paths = all_log_files(base)
    else:
        log_paths = [selected]

    out_dir.mkdir(parents=True, exist_ok=True)

    if len(log_paths) == 1:
        files = plot_file(log_paths[0], out_dir)
    else:
        rows, t_s = read_logs(log_paths)
        stem = "combined_logs"
        results = []
        results.append(
            save_plot(
                out_dir,
                f"{stem}_adxl",
                t_s,
                {
                    "adxl_x": series(rows, "adxl_x"),
                    "adxl_y": series(rows, "adxl_y"),
                    "adxl_z": series(rows, "adxl_z"),
                },
                "ADXL375 (combined logs)",
            )
        )
        results.append(
            save_plot(
                out_dir,
                f"{stem}_imu1_accel",
                t_s,
                {
                    "imu1_ax": series_any(rows, ["imu1_ax_ms2", "imu1_ax"]),
                    "imu1_ay": series_any(rows, ["imu1_ay_ms2", "imu1_ay"]),
                    "imu1_az": series_any(rows, ["imu1_az_ms2", "imu1_az"]),
                },
                "IMU1 Accel (combined logs)",
            )
        )
        results.append(
            save_plot(
                out_dir,
                f"{stem}_imu1_gyro",
                t_s,
                {
                    "imu1_gx": series_any(rows, ["imu1_gx_rads", "imu1_gx"]),
                    "imu1_gy": series_any(rows, ["imu1_gy_rads", "imu1_gy"]),
                    "imu1_gz": series_any(rows, ["imu1_gz_rads", "imu1_gz"]),
                },
                "IMU1 Gyro (combined logs)",
            )
        )
        results.append(
            save_plot(
                out_dir,
                f"{stem}_imu2_accel",
                t_s,
                {
                    "imu2_ax": series_any(rows, ["imu2_ax_ms2", "imu2_ax"]),
                    "imu2_ay": series_any(rows, ["imu2_ay_ms2", "imu2_ay"]),
                    "imu2_az": series_any(rows, ["imu2_az_ms2", "imu2_az"]),
                },
                "IMU2 Accel (combined logs)",
            )
        )
        results.append(
            save_plot(
                out_dir,
                f"{stem}_imu2_gyro",
                t_s,
                {
                    "imu2_gx": series_any(rows, ["imu2_gx_rads", "imu2_gx"]),
                    "imu2_gy": series_any(rows, ["imu2_gy_rads", "imu2_gy"]),
                    "imu2_gz": series_any(rows, ["imu2_gz_rads", "imu2_gz"]),
                },
                "IMU2 Gyro (combined logs)",
            )
        )
        results.append(
            save_plot(
                out_dir,
                f"{stem}_bmp",
                t_s,
                {
                    "temp_c": series(rows, "temp_c"),
                    "press_hpa": series(rows, "press_hpa"),
                    "alt_m": series(rows, "alt_m"),
                },
                "BMP388 (combined logs)",
            )
        )
        results.append(
            save_plot(
                out_dir,
                f"{stem}_flow_valve",
                t_s,
                {
                    "flow_hz": series(rows, "flow_hz"),
                    "flow_lpm": series(rows, "flow_lpm"),
                    "valve": series(rows, "valve"),
                },
                "Flow + Valve (combined logs)",
            )
        )
        files = [p for p in results if p is not None]

    if not files:
        print("No plottable (non-NaN) sensor data found in selected log files")
        return

    print(f"Plotted {len(files)} files from {len(log_paths)} log file(s):")
    for src in log_paths:
        print(f"  source: {src}")
    for p in files:
        print(f"- {p}")


if __name__ == "__main__":
    main()
