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


# BambuStudio GCodeProcessor::ReservedTags, CustomTags and process_tags().
# Keep semantic tags interleaved with commands: moving a role/width tag across
# an extrusion changes its preview even when the ordered tag list is unchanged.
PREVIEW_TAG_PREFIXES = tuple(tag.encode() for tag in (
    "FEATURE:", "WIPE_START", "WIPE_END", "LAYER_HEIGHT:", "LINE_WIDTH:",
    "CHANGE_LAYER", "Z_HEIGHT:", "COLOR_CHANGE", "PAUSE_PRINTING", "CUSTOM_GCODE",
    "_GP_", "WIPE_TOWER_START", "WIPE_TOWER_END", "MACHINE_START_GCODE_END",
    "MACHINE_END_GCODE_START", "NOZZLE_CHANGE_START", "NOZZLE_CHANGE_END",
    "CP_TOOLCHANGE_WIPE", "FLUSH_START", "FLUSH_END", "VFLUSH_START", "VFLUSH_END",
    "SKIPPABLE_START", "SKIPPABLE_END", "SKIPTYPE:", "start printing object",
    "stop printing object", "PA_LINE_CALIBRATION_START", "PA_LINE_CALIBRATION_END",
    "MM3_PER_MM:",
))


def preview_fingerprint(path):
    digest = hashlib.sha256()
    tags = 0
    with Path(path).open("rb") as stream:
        for line in stream:
            command, separator, comment = line.partition(b";")
            if command.strip():
                digest.update(command.strip() + b"\n")
            if separator and comment.lstrip().startswith(PREVIEW_TAG_PREFIXES):
                # Leading whitespace is significant to the native tag parser.
                digest.update(b";" + comment.rstrip(b"\r\n") + b"\n")
                tags += 1
    return {"sha256": digest.hexdigest(), "semantic_tags": tags}


def output_metadata(plate):
    return {key: plate[key] for key in (
        "feature_type_times", "filaments", "main_predication", "total_predication",
        "filament_change_times", "layer_filament_change", "warning_message"
    ) if key in plate}


def equivalence_record(run):
    return {key: run[key] for key in ("commands", "preview", "output_metadata")}


def classify_outputs(calibration, runs):
    # Only independently generated baseline outputs can become references.
    # Nondeterministic controls may reveal a variant after a candidate run.
    references = [equivalence_record(calibration)]
    for run in runs:
        equivalent = equivalence_record(run)
        if run["variant"] == "baseline" and equivalent not in references:
            references.append(equivalent)
    for run in runs:
        equivalent = equivalence_record(run)
        run["output_identical"] = equivalent in references
        run["baseline_output_variant"] = references.index(equivalent) if run["output_identical"] else None
    return references


def run_slice(binary, fixture, directory, data_directory, metrics_runner=None):
    directory.mkdir(parents=True)
    args = [str(binary), "--datadir", str(data_directory), "--debug", "2",
            "--slice", str(fixture["plate"]), "--outputdir", str(directory), fixture["path"]]
    if metrics_runner is not None:
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
        "process_counters": json.loads((directory / "process-metrics.json").read_text()) if metrics_runner else None,
        "elapsed_seconds": elapsed,
        "times_ms": plates[0]["times"],
        "project_times_ms": report.get("times", {}),
        "peak_rss_mb": report.get("peak_rss_mb"),
        "commands": command_fingerprint(gcodes[0]),
        "preview": preview_fingerprint(gcodes[0]),
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
    parser.add_argument("--resume", action="store_true",
                        help="Resume recorded runs only when executable, runner, fixture hashes and round count match")
    args = parser.parse_args()
    if args.rounds < 1 or (args.output.exists() and not args.resume):
        parser.error("rounds must be positive; existing output requires --resume")
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
    output.mkdir(parents=True, exist_ok=args.resume)
    report = {"binaries": {k: {"path": str(v), "sha256": sha256(v)} for k, v in binaries.items()},
              "metrics_runner_sha256": sha256(metrics_runner),
              "primary_metric": "retired_instructions; elapsed times are load/frequency-sensitive observations",
              "fixtures": fixtures, "rounds": args.rounds, "baseline_calibrations": [], "runs": [], "summary": {}}
    if args.resume:
        previous = json.loads((output / "benchmark.json").read_text())
        for key in ("binaries", "metrics_runner_sha256", "fixtures", "rounds"):
            if previous[key] != report[key]:
                parser.error(f"Cannot resume: {key} changed")
        report = previous
    report["verification_protocol"] = "exact commands, interleaved preview tags and metadata against all baseline control runs"
    report["benchmark_script_sha256"] = sha256(__file__)

    def save():
        temporary = output / "benchmark.json.tmp"
        temporary.write_text(json.dumps(report, indent=2) + "\n")
        temporary.replace(output / "benchmark.json")

    save()
    for index, fixture in enumerate(fixtures):
        calibration = next((r for r in report["baseline_calibrations"] if r["fixture"] == fixture["name"]), None)
        if calibration is None:
            calibration, _ = run_slice(binaries["baseline"], fixture,
                                       output / f"fixture-{index}-direct-baseline",
                                       output / "data-baseline")
            calibration.update(fixture=fixture["name"], launcher="direct", measured=False)
            report["baseline_calibrations"].append(calibration)
        fixture_runs = [r for r in report["runs"] if r["fixture"] == fixture["name"]]
        references = classify_outputs(calibration, fixture_runs)
        save()
        for round_index in range(-1, args.rounds):
            warmup = round_index == -1
            order = ("baseline", "candidate") if round_index % 2 == 0 or warmup else ("candidate", "baseline")
            for variant in order:
                if any(r["variant"] == variant and r["round"] == round_index + 1 for r in fixture_runs):
                    continue
                label = "warmup" if warmup else f"round-{round_index + 1}"
                directory = output / f"fixture-{index}-{label}-{variant}"
                run, gcode = run_slice(binaries[variant], fixture, directory, output / f"data-{variant}", metrics_runner)
                run.update(fixture=fixture["name"], variant=variant, warmup=warmup, round=round_index + 1)
                report["runs"].append(run)
                fixture_runs.append(run)
                references = classify_outputs(calibration, fixture_runs)
                save()
                comparison = (f"exact baseline output variant {run['baseline_output_variant']}" if run["output_identical"]
                              else "unverified output; pending remaining baseline controls")
                print(f"{fixture['name']} {label} {variant}: {run['process_counters']['retired_instructions'] / 1e9:.3f} G instructions, "
                      f"{run['elapsed_seconds']:.3f}s observed; {comparison}", flush=True)
                # Retain warmup artifacts for diagnosis, not a large copy per timing run.
                if not warmup and run["output_identical"]:
                    gcode.unlink()
        if any(not r["output_identical"] for r in fixture_runs):
            raise RuntimeError(f"Candidate output not reproduced by any baseline control: {fixture['name']}")
        for directory in output.glob(f"fixture-{index}-round-*"):
            for gcode in directory.glob("*.gcode"):
                gcode.unlink()
        measured = [r for r in report["runs"] if r["fixture"] == fixture["name"] and not r["warmup"]]
        summary = {}
        summary["baseline_output_variants"] = len(references)
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
