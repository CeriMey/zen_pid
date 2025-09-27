#!/usr/bin/env python3
"""Generate multi-plant validation runs for AdaptivePID.

This script compiles the C++ validation harness and executes it on a
collection of predefined first-order-plus-dead-time (FOPDT) plants. Each
scenario has its own setpoint profile, disturbance plan, and stochastic
seed so we can study convergence behaviour under a variety of operating
conditions. Detailed CSV logs are stored in ``tests/data/suite`` along
with a JSON manifest summarising the configuration and closed-loop
metrics gathered for every test.
"""
from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List

REPO_ROOT = Path(__file__).resolve().parents[1]
TEST_DIR = REPO_ROOT / "tests"
DATA_DIR = TEST_DIR / "data" / "suite"
BINARY_PATH = TEST_DIR / "pid_validation"

DEFAULT_STEPS = 3000


@dataclass
class Excitation:
    duration: float
    amplitude: float
    period: float


@dataclass
class Scenario:
    name: str
    description: str
    plant: Dict[str, float]
    Ts: float
    steps: int = DEFAULT_STEPS
    noise_std: float = 0.01
    seed: int = 42
    u_min: float = -5.0
    u_max: float = 5.0
    imc_lambda: float = 0.0
    excitation: Excitation | None = None
    setpoints: List[Dict[str, float]] = None
    disturbances: List[Dict[str, float]] = None

    def to_manifest(self) -> Dict[str, object]:
        manifest = {
            "name": self.name,
            "description": self.description,
            "plant": self.plant,
            "Ts": self.Ts,
            "steps": self.steps,
            "noise_std": self.noise_std,
            "seed": self.seed,
            "u_min": self.u_min,
            "u_max": self.u_max,
            "setpoints": self.setpoints or [],
            "disturbances": self.disturbances or [],
        }
        if self.imc_lambda > 0.0:
            manifest["imc_lambda"] = self.imc_lambda
        if self.excitation:
            manifest["excitation"] = asdict(self.excitation)
        return manifest


SCENARIOS: List[Scenario] = [
    Scenario(
        name="fast_thruster",
        description="Fast pneumatic actuator with short dead time and narrow actuation limits.",
        plant={"K": 0.9, "tau": 0.25, "L": 0.04},
        Ts=0.02,
        noise_std=0.004,
        seed=101,
        u_min=-2.0,
        u_max=2.0,
        excitation=Excitation(duration=3.0, amplitude=0.8, period=0.2),
        setpoints=[
            {"time": 0.0, "value": 0.0},
            {"time": 6.0, "value": 1.0},
            {"time": 18.0, "value": -0.2},
            {"time": 30.0, "value": 0.7},
            {"time": 42.0, "value": 0.0},
            {"time": 52.0, "value": 0.5},
        ],
        disturbances=[
            {"time": 0.0, "value": 0.0},
            {"time": 28.0, "value": 0.12},
            {"time": 45.0, "value": -0.08},
        ],
    ),
    Scenario(
        name="thermal_plate",
        description="Medium-speed heating loop subject to slow load variations.",
        plant={"K": 1.7, "tau": 1.0, "L": 0.3},
        Ts=0.05,
        noise_std=0.01,
        seed=202,
        u_min=-3.0,
        u_max=3.0,
        excitation=Excitation(duration=6.0, amplitude=1.0, period=0.5),
        setpoints=[
            {"time": 0.0, "value": 0.0},
            {"time": 15.0, "value": 1.2},
            {"time": 60.0, "value": 0.6},
            {"time": 95.0, "value": 1.5},
            {"time": 125.0, "value": 1.0},
        ],
        disturbances=[
            {"time": 0.0, "value": 0.0},
            {"time": 70.0, "value": 0.18},
        ],
    ),
    Scenario(
        name="slow_tank",
        description="Large storage tank with sluggish dynamics and negative disturbance step.",
        plant={"K": 2.2, "tau": 3.5, "L": 0.8},
        Ts=0.1,
        noise_std=0.008,
        seed=303,
        u_min=-4.0,
        u_max=4.0,
        excitation=Excitation(duration=12.0, amplitude=0.7, period=0.6),
        setpoints=[
            {"time": 0.0, "value": 0.0},
            {"time": 50.0, "value": 1.0},
            {"time": 150.0, "value": 1.6},
            {"time": 230.0, "value": 0.8},
            {"time": 270.0, "value": 1.3},
        ],
        disturbances=[
            {"time": 0.0, "value": 0.0},
            {"time": 180.0, "value": -0.25},
        ],
    ),
    Scenario(
        name="precision_stage",
        description="High-precision positioning axis with tighter noise and asymmetric setpoints.",
        plant={"K": 1.3, "tau": 0.6, "L": 0.18},
        Ts=0.04,
        noise_std=0.006,
        seed=404,
        u_min=-2.5,
        u_max=2.5,
        excitation=Excitation(duration=4.5, amplitude=1.1, period=0.25),
        setpoints=[
            {"time": 0.0, "value": 0.0},
            {"time": 12.0, "value": 0.5},
            {"time": 36.0, "value": 1.2},
            {"time": 58.0, "value": 0.3},
            {"time": 78.0, "value": -0.6},
            {"time": 102.0, "value": 0.1},
        ],
        disturbances=[
            {"time": 0.0, "value": 0.0},
            {"time": 80.0, "value": 0.22},
        ],
    ),
]


def compile_harness() -> None:
    """Build the validation executable in-place."""
    cmd = [
        "g++",
        "-std=c++17",
        "-O2",
        f"-I{REPO_ROOT}",
        str(TEST_DIR / "pid_validation.cpp"),
        "-o",
        str(BINARY_PATH),
    ]
    subprocess.run(cmd, check=True)


def ensure_time_horizon_ok(scenario: Scenario) -> None:
    duration = scenario.steps * scenario.Ts
    for entry in (scenario.setpoints or []):
        if entry["time"] > duration + 1e-9:
            raise ValueError(
                f"Setpoint at t={entry['time']}s exceeds simulation duration {duration:.2f}s"
            )
    for entry in (scenario.disturbances or []):
        if entry["time"] > duration + 1e-9:
            raise ValueError(
                f"Disturbance at t={entry['time']}s exceeds simulation duration {duration:.2f}s"
            )


def build_args(scenario: Scenario, csv_path: Path) -> List[str]:
    args = [
        "--K",
        str(scenario.plant["K"]),
        "--tau",
        str(scenario.plant["tau"]),
        "--delay",
        str(scenario.plant["L"]),
        "--Ts",
        str(scenario.Ts),
        "--steps",
        str(scenario.steps),
        "--noise-std",
        str(scenario.noise_std),
        "--seed",
        str(scenario.seed),
        "--umin",
        str(scenario.u_min),
        "--umax",
        str(scenario.u_max),
        "--csv",
        str(csv_path),
    ]
    if scenario.imc_lambda > 0.0:
        args.extend(["--imc-lambda", str(scenario.imc_lambda)])
    if scenario.excitation:
        args.extend(
            [
                "--excitation-duration",
                str(scenario.excitation.duration),
                "--excitation-amplitude",
                str(scenario.excitation.amplitude),
                "--excitation-period",
                str(scenario.excitation.period),
            ]
        )
    else:
        args.append("--no-excitation")

    for tv in scenario.setpoints or []:
        args.extend(["--sp", f"{tv['time']}:{tv['value']}"])
    for tv in scenario.disturbances or []:
        args.extend(["--dist", f"{tv['time']}:{tv['value']}"])
    return args


def parse_metrics(stdout: str) -> Dict[str, float]:
    metrics: Dict[str, float] = {}
    for line in stdout.splitlines():
        line = line.strip()
        if line.startswith("Qualité (controller):"):
            parts = line.split(",")
            for part in parts:
                if "FIT=" in part:
                    metrics["controller_fit_percent"] = float(part.split("FIT=")[-1].split()[0])
                elif "RMSE=" in part:
                    metrics["controller_fit_rmse"] = float(part.split("RMSE=")[-1].strip())
        elif line.startswith("Erreurs boucle fermée"):
            # Format: Erreurs ...: RMSE=..., MAE=..., IAE=... s, overshoot max=...
            entries = line.split(",")
            for entry in entries:
                if "RMSE=" in entry and "IAE" not in entry:
                    metrics["closed_loop_rmse"] = float(entry.split("RMSE=")[-1].strip())
                elif "MAE=" in entry:
                    metrics["closed_loop_mae"] = float(entry.split("MAE=")[-1].strip())
                elif "IAE=" in entry:
                    value = entry.split("IAE=")[-1].split()[0]
                    metrics["closed_loop_iae"] = float(value)
                elif "overshoot max=" in entry:
                    metrics["closed_loop_max_overshoot"] = float(entry.split("overshoot max=")[-1].strip())
    return metrics


def extract_report(stdout: str) -> Dict[str, object]:
    marker = "Rapport JSON:"
    idx = stdout.find(marker)
    if idx == -1:
        raise RuntimeError("Unable to locate controller JSON report in output")
    json_segment = stdout[idx + len(marker) :].strip()
    first_line = json_segment.splitlines()[0]
    return json.loads(first_line)


def run_scenario(scenario: Scenario) -> Dict[str, object]:
    ensure_time_horizon_ok(scenario)
    csv_path = DATA_DIR / f"{scenario.name}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    args = [str(BINARY_PATH)] + build_args(scenario, csv_path)
    result = subprocess.run(args, check=True, capture_output=True, text=True)

    metrics = parse_metrics(result.stdout)
    report = extract_report(result.stdout)

    entry = scenario.to_manifest()
    entry.update(
        {
            "csv": str(csv_path.relative_to(REPO_ROOT)),
            "metrics": metrics,
            "estimated_model": report.get("model", {}),
            "estimated_pid": report.get("pid", {}),
            "fit": report.get("fit", {}),
            "options": report.get("options", {}),
        }
    )
    return entry


def main() -> int:
    compile_harness()
    manifest: List[Dict[str, object]] = []

    for scenario in SCENARIOS:
        duration = scenario.steps * scenario.Ts
        print(
            f"[suite] Running {scenario.name} ({scenario.description}) with {scenario.steps} iterations"
            f" over {duration:.1f}s..."
        )
        manifest.append(run_scenario(scenario))

    manifest_path = DATA_DIR / "suite_manifest.json"
    with manifest_path.open("w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2)
        fh.write("\n")

    print(f"[suite] Manifest saved to {manifest_path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        sys.stderr.write(exc.stdout or "")
        sys.stderr.write(exc.stderr or "")
        raise
