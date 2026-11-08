#!/usr/bin/env python3
"""Run alternating full-pipeline CLI slices and verify their generated commands.

Fixture manifest: [{"name": "assembly", "path": "/path/to/model.3mf", "plate": 1}]
Uses isolated data directories; never modifies the supplied projects or profiles.
"""

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import time


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def command_fingerprint(path):
    digest = hashlib.sha256()
    count = 0
    with Path(path).open("rb") as stream:
        for line in stream:
            command = line.split(b";", 1)[0].strip()
            if command:
                digest.update(command + b"\n")
                count += 1
    return {"sha256": digest.hexdigest(), "commands": count}


def output_metadata(plate):
    return {key: plate[key] for key in (
        "feature_type_times", "filaments", "main_predication", "total_predication",
        "filament_change_times", "layer_filament_change", "warning_message"
    ) if key in plate}


def run_slice(binary, fixture, directory, data_directory, metrics_runner):
    directory.mkdir(parents=True)
    args = [str(binary), "--datadir", str(data_directory), "--debug", "2",
            "--slice", str(fixture["plate"]), "--outputdir", str(directory), fixture["path"]]
    args = [str(metrics_runner), str(directory / "process-metrics.json"), "--", *args]
    started = time.monotonic()
    with (directory / "slice.log").open("wb") as log:
        result = subprocess.run(args, stdout=log, stderr=subprocess.STDOUT)
    elapsed = time.monotonic() - started
    if result.returncode:
        raise RuntimeError(f"Slicing failed ({result.returncode}); see {directory / 'slice.log'}")
    report = json.loads((directory / "result.json").read_text())
    plates = report["sliced_plates"]
    if len(plates) != 1:
        raise RuntimeError(f"Expected one sliced plate in {directory}")
    gcodes = list(directory.glob("*.gcode"))
    if len(gcodes) != 1:
        raise RuntimeError(f"Expected one G-code file in {directory}")
    return {
        "process_counters": json.loads((directory / "process-metrics.json").read_text()),
        "elapsed_seconds": elapsed,
        "times_ms": plates[0]["times"],
        "project_times_ms": report.get("times", {}),
        "peak_rss_mb": report.get("peak_rss_mb"),
        "commands": command_fingerprint(gcodes[0]),
        "output_metadata": output_metadata(plates[0]),
    }, gcodes[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--fixtures", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metrics-runner", type=Path, required=True,
                        help="Compiled measure_macos_process.c; retired instructions are the primary work metric")
    parser.add_argument("--rounds", type=int, default=5)
    args = parser.parse_args()
    if args.rounds < 1 or args.output.exists():
        parser.error("rounds must be positive and output must be a new directory")
    binaries = {"baseline": args.baseline.resolve(), "candidate": args.candidate.resolve()}
    metrics_runner = args.metrics_runner.resolve()
    fixtures = json.loads(args.fixtures.read_text())
    if not fixtures or len({f["name"] for f in fixtures}) != len(fixtures):
        parser.error("fixture names must be unique and at least one fixture is required")
    for fixture in fixtures:
        fixture["path"] = str(Path(fixture["path"]).resolve())
        fixture["plate"] = int(fixture.get("plate", 1))
        if fixture["plate"] < 1:
            parser.error("each fixture must select one positive plate index")
        fixture["sha256"] = sha256(fixture["path"])
    output = args.output.resolve()
    output.mkdir(parents=True)
    report = {"binaries": {k: {"path": str(v), "sha256": sha256(v)} for k, v in binaries.items()},
              "metrics_runner_sha256": sha256(metrics_runner),
              "primary_metric": "retired_instructions; elapsed times are load/frequency-sensitive observations",
              "fixtures": fixtures, "rounds": args.rounds, "runs": [], "summary": {}}

    def save():
        (output / "benchmark.json").write_text(json.dumps(report, indent=2) + "\n")

    save()
    for index, fixture in enumerate(fixtures):
        reference = None
        for round_index in range(-1, args.rounds):
            warmup = round_index == -1
            order = ("baseline", "candidate") if round_index % 2 == 0 or warmup else ("candidate", "baseline")
            for variant in order:
                label = "warmup" if warmup else f"round-{round_index + 1}"
                directory = output / f"fixture-{index}-{label}-{variant}"
                run, gcode = run_slice(binaries[variant], fixture, directory, output / f"data-{variant}", metrics_runner)
                run.update(fixture=fixture["name"], variant=variant, warmup=warmup, round=round_index + 1)
                equivalent = {key: run[key] for key in ("commands", "output_metadata")}
                if reference is None:
                    reference = equivalent
                run["output_identical"] = equivalent == reference
                report["runs"].append(run)
                save()
                if not run["output_identical"]:
                    raise RuntimeError(f"Output differs from baseline: {directory}")
                print(f"{fixture['name']} {label} {variant}: {run['process_counters']['retired_instructions'] / 1e9:.3f} G instructions, "
                      f"{run['elapsed_seconds']:.3f}s observed; commands identical", flush=True)
                # Retain warmup artifacts for diagnosis, not a large copy per timing run.
                if not warmup:
                    gcode.unlink()
        measured = [r for r in report["runs"] if r["fixture"] == fixture["name"] and not r["warmup"]]
        summary = {}
        for variant in binaries:
            runs = [r for r in measured if r["variant"] == variant]
            elapsed = [r["elapsed_seconds"] for r in runs]
            instructions = [r["process_counters"]["retired_instructions"] for r in runs]
            summary[variant] = {"median_seconds": statistics.median(elapsed), "worst_seconds": max(elapsed),
                                "median_instructions": statistics.median(instructions),
                                "min_instructions": min(instructions), "max_instructions": max(instructions),
                                "max_physical_bytes": max(r["process_counters"]["peak_physical_bytes"] for r in runs),
                                "median_stage_ms": {key: statistics.median(r["times_ms"][key] for r in runs)
                                                    for key in runs[0]["times_ms"]}}
        summary["instruction_reduction_fraction"] = 1 - summary["candidate"]["median_instructions"] / summary["baseline"]["median_instructions"]
        summary["observed_elapsed_ratio"] = summary["baseline"]["median_seconds"] / summary["candidate"]["median_seconds"]
        summary["paired_instruction_ratios"] = [
            next(r["process_counters"]["retired_instructions"] for r in measured if r["variant"] == "candidate" and r["round"] == n) /
            next(r["process_counters"]["retired_instructions"] for r in measured if r["variant"] == "baseline" and r["round"] == n)
            for n in range(1, args.rounds + 1)
        ]
        report["summary"][fixture["name"]] = summary
        save()
    # Detect accidental edits or cloud changes to the input during a long run.
    for fixture in fixtures:
        if sha256(fixture["path"]) != fixture["sha256"]:
            raise RuntimeError(f"Fixture changed during benchmark: {fixture['name']}")


if __name__ == "__main__":
    main()
