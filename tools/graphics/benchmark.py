#!/usr/bin/env python3
# Copyright 2026 Sharpa
# SPDX-License-Identifier: Apache-2.0
"""Maintainer acceptance runner. Start the README router and prepare vcan first.

Uses local simulation only. Each invocation runs three static and three SDK-motion
measurements. It never installs drivers, changes rendering environment variables,
or kills existing processes. Raw reports belong outside tracked release files.
"""
import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def output(*command):
    return subprocess.check_output(command, text=True).strip()


def run(args):
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=False)
    driver = output("nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader")
    if driver != "NVIDIA GeForce RTX 4070 Ti SUPER, " + args.driver:
        raise RuntimeError("Unexpected actual GPU/driver: " + driver)
    if output("uname", "-r") != "6.8.0-138-generic":
        raise RuntimeError("Keep the agreed kernel 6.8.0-138-generic")
    kernel_driver = Path("/proc/driver/nvidia/version").read_text()
    if args.driver not in kernel_driver:
        raise RuntimeError("Kernel/user driver versions differ")
    sdk = args.sdk_root.resolve()
    # Source the release's setup script using positional arguments, not shell interpolation.
    sdk_env = json.loads(output("bash", "-c",
        'source "$1/setup.bash" >/dev/null && python3 -c '\
        "'import os,json; print(json.dumps(dict(os.environ)))'", "sdk-env", str(sdk)))
    simulator = args.simulator.resolve()
    config = args.config.resolve()
    manifest = {"gpu_driver": driver, "kernel": output("uname", "-r"),
                "session_type": os.environ.get("XDG_SESSION_TYPE", "unknown"),
                "display_modes": output("xrandr", "--current") if shutil.which("xrandr") else "unavailable",
                "kernel_driver": kernel_driver, "commit": output("git", "-C", str(ROOT), "rev-parse", "HEAD"),
                "source_diff_sha256": hashlib.sha256(subprocess.check_output(["git", "-C", str(ROOT), "diff", "--binary"])).hexdigest(),
                "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
                "simulator_sha256": hashlib.sha256(simulator.read_bytes()).hexdigest(),
                "config": str(config), "runs": []}
    try:
        for workload in (("static", "motion") if args.workload == "both" else (args.workload,)):
            for repetition in range(1, 4):
                label = f"{workload}-{repetition}"
                report_path = destination / (label + ".json")
                simulator_log = open(destination / (label + "-simulator.log"), "w")
                motion_log = open(destination / (label + "-sdk.log"), "w")
                process = None
                motion = None
                start = time.monotonic()
                try:
                    process = subprocess.Popen([str(simulator), "--config", str(config),
                        "--with-platform", "--benchmark-report", str(report_path)],
                        cwd=ROOT, stdout=simulator_log, stderr=subprocess.STDOUT)
                    # Platform startup is asynchronous. Preview waits for fresh Master/state.
                    time.sleep(3)
                    if process.poll() is not None:
                        raise RuntimeError(label + ": simulator exited; inspect its log")
                    ready_deadline = time.monotonic() + 35
                    while True:
                        preview = subprocess.run([sys.executable, str(ROOT / "examples/sdk/whole_body_motion.py"), "--preview"],
                            env=sdk_env, capture_output=True, text=True, timeout=20)
                        motion_log.write(preview.stdout + preview.stderr)
                        motion_log.flush()
                        if preview.returncode:
                            raise RuntimeError(label + ": SDK preview failed")
                        if "Master=STANDBY " in preview.stdout:
                            break
                        if "Master=INIT " not in preview.stdout or time.monotonic() >= ready_deadline:
                            raise RuntimeError(label + ": fresh STANDBY required")
                        time.sleep(0.5)
                    while not report_path.exists():
                        if process.poll() is not None or time.monotonic() - start > 150:
                            raise RuntimeError(label + ": report missing or simulator stopped")
                        if workload == "motion" and motion is None:
                            motion = subprocess.Popen([sys.executable, str(ROOT / "examples/sdk/whole_body_motion.py"),
                                "--hold-duration", "2"], env=sdk_env, stdout=motion_log, stderr=subprocess.STDOUT)
                        if motion is not None and motion.poll() is not None:
                            if motion.returncode:
                                raise RuntimeError(label + ": SDK motion failed")
                            motion = None
                        time.sleep(0.2)
                    if motion is not None:
                        # Complete the current motion and stop its task before simulation shutdown.
                        if motion.wait(timeout=65):
                            raise RuntimeError(label + ": final SDK motion failed")
                        motion = None
                    report = json.loads(report_path.read_text())
                    if not report["gl_version"].endswith(args.driver):
                        raise RuntimeError(label + ": GL driver differs from NVML/kernel")
                    manifest["runs"].append({"workload": workload, "repetition": repetition,
                                              "report": report_path.name, "passed": report["passed"]})
                    print(label, "FPS=", round(report["average_fps"], 2),
                          "p95_ms=", round(report["frame_time_p95_ms"], 2),
                          "RTF=", round(report["realtime_ratio"], 4),
                          "passed=", report["passed"], flush=True)
                finally:
                    cleanup_errors = []
                    try:
                        if motion is not None and motion.poll() is None:
                            motion.send_signal(signal.SIGINT)
                            try:
                                motion.wait(timeout=35)
                            except subprocess.TimeoutExpired:
                                motion.kill()
                                motion.wait(timeout=10)
                                cleanup_errors.append("SDK did not stop within 35s")
                    finally:
                        try:
                            if process is not None:
                                if process.poll() is None:
                                    process.send_signal(signal.SIGINT)
                                try:
                                    process.wait(timeout=20)
                                except subprocess.TimeoutExpired:
                                    process.kill()
                                    process.wait(timeout=10)
                                    cleanup_errors.append("simulator did not stop within 20s")
                                if process.returncode != 0:
                                    cleanup_errors.append("simulator did not exit cleanly")
                        finally:
                            simulator_log.close()
                            motion_log.close()
                    if cleanup_errors:
                        raise RuntimeError(label + ": " + "; ".join(cleanup_errors))
    except BaseException as error:
        manifest["error"] = type(error).__name__ + ": " + str(error)
        raise
    finally:
        manifest["passed"] = "error" not in manifest and len(manifest["runs"]) == (6 if args.workload == "both" else 3) and all(r["passed"] for r in manifest["runs"])
        (destination / "summary.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return 0 if manifest["passed"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workload", choices=("both", "static", "motion"), default="both")
    parser.add_argument("--driver", required=True, choices=("570.211.01", "580.178.04", "595.99.02"))
    parser.add_argument("--sdk-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "sharpa_mujoco.conf")
    parser.add_argument("--simulator", type=Path,
                        default=ROOT / "build-linux/apps/simulator/sharpa_mujoco_simulator")
    try:
        sys.exit(run(parser.parse_args()))
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
