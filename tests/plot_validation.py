#!/usr/bin/env python3
"""Visualise les enregistrements générés par tests/pid_validation."""
import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt


@dataclass
class Sample:
    iteration: int
    time: float
    setpoint: float
    y_true: float
    y_meas: float
    u: float
    u_ff: float
    model_valid: bool
    est_K: float
    est_tau: float
    est_L: float
    delay_samples: float
    fit_percent: float
    fit_rmse: float
    Kp: float
    Ti: float
    Td: float


def read_csv(path: Path) -> List[Sample]:
    samples: List[Sample] = []
    with path.open() as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            samples.append(
                Sample(
                    iteration=int(row["iteration"]),
                    time=float(row["time"]),
                    setpoint=float(row["setpoint"]),
                    y_true=float(row["y_true"]),
                    y_meas=float(row["y_meas"]),
                    u=float(row["u"]),
                    u_ff=float(row["u_ff"]),
                    model_valid=row["model_valid"] == "1",
                    est_K=float(row["est_K"]),
                    est_tau=float(row["est_tau"]),
                    est_L=float(row["est_L"]),
                    delay_samples=float(row["delay_samples"]),
                    fit_percent=float(row["fit_percent"]),
                    fit_rmse=float(row["fit_rmse"]),
                    Kp=float(row["Kp"]),
                    Ti=float(row["Ti"]),
                    Td=float(row["Td"]),
                )
            )
    if not samples:
        raise ValueError(f"Le fichier {path} ne contient aucune donnée")
    return samples


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Trace les résultats d'une simulation AdaptivePID")
    parser.add_argument("csv", type=Path, help="Chemin du log CSV produit par pid_validation")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Chemin du fichier image (PNG) à générer. Si omis, affiche la fenêtre interactive.",
    )
    parser.add_argument(
        "--title",
        default="AdaptivePID - suivi et convergence",
        help="Titre à afficher sur la figure",
    )
    return parser


def plot(samples: List[Sample], title: str):
    times = [s.time for s in samples]

    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    fig.suptitle(title)

    # Suivi boucle fermée
    axes[0].plot(times, [s.setpoint for s in samples], label="Consigne", linestyle="--")
    axes[0].plot(times, [s.y_true for s in samples], label="Sortie vraie")
    axes[0].plot(times, [s.y_meas for s in samples], label="Mesure bruyante", alpha=0.6)
    axes[0].set_ylabel("Valeur")
    axes[0].legend(loc="best")
    axes[0].grid(True)

    # Estimation FOPDT
    axes[1].plot(times, [s.est_K for s in samples], label="K estimé")
    axes[1].plot(times, [s.est_tau for s in samples], label="tau estimé (s)")
    axes[1].plot(times, [s.est_L for s in samples], label="L estimé (s)")
    axes[1].set_ylabel("Paramètres modèle")
    axes[1].legend(loc="best")
    axes[1].grid(True)

    # Gains PID + qualité
    ax2 = axes[2]
    ax2.plot(times, [s.Kp for s in samples], label="Kp")
    ax2.plot(times, [s.Ti for s in samples], label="Ti (s)")
    ax2.plot(times, [s.Td for s in samples], label="Td (s)")
    ax2.set_ylabel("Gains PID")
    ax2.grid(True)

    ax3 = ax2.twinx()
    ax3.plot(times, [s.fit_percent for s in samples], label="FIT %", color="tab:purple", linestyle=":")
    ax3.set_ylabel("Qualité d'identification (%)")

    # Combiner légendes des deux axes
    lines, labels = ax2.get_legend_handles_labels()
    lines2, labels2 = ax3.get_legend_handles_labels()
    ax2.legend(lines + lines2, labels + labels2, loc="best")

    ax2.set_xlabel("Temps (s)")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


def main():
    parser = build_parser()
    args = parser.parse_args()
    samples = read_csv(args.csv)
    fig = plot(samples, args.title)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=200)
        print(f"Figure sauvegardée dans {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
