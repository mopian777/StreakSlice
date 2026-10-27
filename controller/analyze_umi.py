"""After-only analysis for 1100 UMi-corrected fixed-model geometry replays.

This adapter reuses only the former study's pure numerical analysis functions.
It never reads its outcomes, recovery manifests or retry routing. No simulation,
model loading, optimizer update, checkpoint choice, retry or plotting is done.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "cp11_main_comparison_20261017"
SOURCE_PLAN = SOURCE / "plan.json"
ENVIRONMENT = "umi_compliant_geometry_20261019"
BINARY = HERE / "build_001" / "deadline_sim"
INPUTS, EVALUATIONS, TRAINING, UPDATES, RETRIES = 100, 1100, 0, 0, 0

_spec = importlib.util.spec_from_file_location("umi_original_pure_analysis", SOURCE / "analyze1292.py")
legacy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(legacy)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def read_object(path):
    value = read_json(path)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object: {path}")
    return value


def path_key(value):
    return str(value).replace("\\", "/").casefold()


def validate_jobs(jobs, source_jobs=None):
    """Check only the ordinary explicit-plan identities; no new frozen contract."""
    if not isinstance(jobs, list) or len(jobs) != EVALUATIONS:
        raise ValueError("Exactly 1100 fixed-model geometry replay jobs are required")
    if source_jobs is None:
        source_jobs = read_object(SOURCE_PLAN)["jobs"]
    _, original_evaluations = legacy.validate_jobs(source_jobs)
    by_id = {}
    for job in jobs:
        if not isinstance(job, dict) or type(job.get("id")) is not int:
            raise ValueError("Each replay needs an integer job ID")
        job_id = job["id"]
        source_id = job.get("source_id")
        if (job_id in by_id or not 0 <= job_id < EVALUATIONS
                or type(source_id) is not int or source_id != job_id + 192
                or source_id not in original_evaluations):
            raise ValueError("Replay IDs must be unique 0..1099, mapped to original 192..1291")
        expected = dict(original_evaluations[source_id], id=job_id,
                        source_id=source_id, environment=ENVIRONMENT)
        if job != expected:
            raise ValueError("Replay may change only ID and explicit environment/source provenance")
        by_id[job_id] = job
    if set(by_id) != set(range(EVALUATIONS)):
        raise ValueError("Incomplete 1100-cell geometry replay plan")
    return by_id


def directory_job_id(path):
    name = path.parent.name
    if not name.startswith("j") or not name[1:].isdigit() or name != f"j{int(name[1:]):04d}":
        raise ValueError(f"Noncanonical episode directory; retries are not authorized: {path}")
    return int(name[1:])


def load_evaluation(run, jobs, source_jobs=None):
    planned = validate_jobs(jobs, source_jobs)
    paths = {}
    for path in (run / "episodes").glob("*/result.json"):
        job_id = directory_job_id(path)
        if job_id in paths:
            raise ValueError("Duplicate geometry replay result")
        paths[job_id] = path
    if set(paths) != set(planned) or list((run / "episodes").glob("*/failure.json")):
        raise ValueError("Analysis requires all 1100 new successful observations and no failed/retry attempts")
    inputs, evaluation = {}, []
    for job_id in sorted(planned):
        job, path = planned[job_id], paths[job_id]
        folder = run / "episodes" / f"j{job_id:04d}"
        raw = read_object(path)
        if raw.get("job") != job or read_object(folder / "attempt.json").get("job") != job:
            raise ValueError(f"Replay attempt/result/planned identity mismatch: {path}")
        if path_key(raw.get("simulator")) != path_key(BINARY):
            raise ValueError(f"Result does not identify the isolated UMi-corrected binary: {path}")
        native = read_object(folder / "native_exit.json")
        worker = read_object(run / "exits" / f"worker_{job_id:04d}.json")
        processes = native.get("processes", [])
        if (worker.get("id") != job_id or worker.get("job") != job
                or worker.get("actual_exit_code") != 0
                or native.get("all_exited") is not True
                or native.get("simulation_dispatches") != 1
                or native.get("worker_exit_code") != 0
                or len(processes) != 1 or processes[0].get("actual_exit_code") != 0):
            raise ValueError(f"Actual worker/native one-dispatch exit0 evidence is incomplete: {path}")
        attempts = legacy.number(raw.get("native_attempts"), str(path))
        if attempts <= 0 or attempts != legacy.number(raw.get("native_completions"), str(path)):
            raise ValueError(f"Incomplete actual native decision accounting: {path}")
        terminal = raw.get("result")
        if not isinstance(terminal, dict):
            raise ValueError(f"Missing terminal metrics: {path}")
        harm = legacy.number(raw.get("weighted_absolute_harm"), str(path))
        if harm < 0 or harm != legacy.number(terminal.get("weighted_absolute_harm"), str(path)):
            raise ValueError(f"Invalid or mismatched source/terminal H: {path}")
        for field in ("all_offered_denominator", "original_quota_deadline_and_telemetry_audits_passed"):
            if terminal.get(field) is not True:
                raise ValueError(f"Existing all-offered/telemetry audit missing or failed: {path}")
        unchanged = raw.get("policy_parameters_unchanged", raw.get("parameters_unchanged"))
        if (unchanged is not True or raw.get("training_updates_during_collection") != 0
                or raw.get("validation_used_for_training") is not False):
            raise ValueError(f"Replay is not recorded as fixed-policy zero-update evaluation: {path}")
        signature = terminal.get("all_ingress_signature")
        if not isinstance(signature, str) or not signature:
            raise ValueError(f"Actual ingress pairing is UNKNOWN: {path}")
        offered = tuple(legacy.number(terminal.get(f"{name}_{suffix}"), f"{name}_{suffix}")
                        for name in legacy.SLICES for suffix in ("offered_packets", "offered_bytes"))
        pairing = (signature, offered)
        index = job["input_index"]
        if index in inputs and inputs[index] != pairing:
            raise ValueError(f"Ingress/offered population differs across eleven policies: input {index}")
        inputs[index] = pairing
        evaluation.append({"job": job, "weighted_absolute_harm": harm,
                           "all_terminal_metrics": terminal, "result_reference": str(path.resolve())})
    if len(inputs) != INPUTS:
        raise ValueError("One hundred complete paired input scenarios are required")
    return evaluation


def interpretation():
    """Keep the original scientific limitations, without transplanting old attempts."""
    return [
        "This batch is a historical-trained fixed-model UMi-corrected geometry replay: 1100 evaluations, nine fixed cp11 learned models and two actual deterministic rules. There is no new training, optimizer update, model selection, recovery or retry in this batch.",
        "The same previously evaluated 100 input scenarios (50 burst/50 compound; seeds823200..823299 and sampling1962000+i) are replayed with corrected geometry. They are not newly unseen inputs. There are 100 distinct paired inputs within this matrix, not1100 or300 independent inputs; old and corrected observations are not pooled into200 independent inputs.",
        "Only the new isolated binary changes the intended evaluation geometry: gNB height10m, UE height1.5m and the original 3x3m UE offsets shifted20m in x. Training remains historical; this does not claim end-to-end UMi-compliant training or a calibrated urban deployment.",
        "RU64, original-tail-Q and matched DirPPO each retain their three fixed initialized cp11 models, with no checkpoint selection. Main-method selection followed older evidence. Repeated-input geometry reassessment is not an originally prospective whole-AND confirmation.",
        "Tail-Q is the same-parent/same-behavior-data local objective comparison. Matched DirPPO has its own historical on-policy data and matching interaction/epoch counts; that does not imply matching parameter count, FLOPs or action parameterization.",
        "Guarded Equal-projection and Guarded queue-weighted anchor are local adaptations within the shared candidates and protection interface. They are not exact reproductions of published RR/PBA/standard MaxWeight. A single rule vector reused for descriptive seed pairings is not three actual rule evaluations per input.",
        "The primary H-CVaR95 is computed over100 inputs separately for each fixed model, then equally averaged over the three fixed initializations (one vector for each rule). Tail mass is five observations per model. A pooled300-cell mixture CVaR is a different estimand.",
        "Mean H is secondary and cannot replace an unestablished CVaR advantage. Every contrast is RU64 minus control; negative H differences favor RU64. The same2000 bootstrap replicates with seed88581 resample50/50 input strata using shared indices across all eleven policies. Pointwise approximate95% percentile intervals condition on these three fixed models, with no training-seed-population or simultaneous multi-metric guarantee.",
        "All three seeds and all favorable/equal/unfavorable outcomes are retained. Missing/null recovery, censoring and received-delay values are not zero. All URLLC/eMBB/mMTC timely, deadline-failure, actual-late, undelivered, expiry/overflow, received-delay and streak/recovery quantities are retained rather than replacing them by composite H.",
        "Timely fractions use the existing deadline population, while byte-failure/cohort H and direct scored streaks use their existing scored populations. Conditional received delays use TX[0.4,38.4)s and reception by40s. The received_CVaR95_ms source field is the per-episode rounded top5% received-delay tail mean, not fractional-mass CVaR for arbitrary packet count. Undelivered-at-end uses full episode ingress with no reception by40s, including unexpired packets; it is not the TX-truncated latency-undelivered count or a deadline-miss count.",
        "The original two offline-recovered training records and two failed original rule attempts plus authorized retries remain historical source provenance only. None belongs to this batch's dispatches, result loading or endpoints, and no old outcome substitutes a corrected-geometry result.",
        "Passing the propagation model's geometric parameter range does not prove multi-cell, mobile, E2/OTA, real-time or hard-SLA performance, or isolate causal guard/rescue benefits. Completion, lower loss, favorable point estimates or policy separation alone are not method success. B/C are not started.",
    ]


def concise_summary(stats):
    # Reuse the existing four-contrast formatting only; replace every old-run
    # bookkeeping and interpretation field rather than implying old jobs ran.
    old = legacy.concise_summary(stats)
    fields = ("group_summaries", "primary_composite_H_CVaR95", "secondary_mean_H",
              "all_slice_benefits_and_costs_descriptive", "paired_direction_counts", "bootstrap")
    result = {key: old[key] for key in fields}
    result.update(
        status="completed_umi_compliant_geometry_fixed1100_replay_analysis",
        environment=ENVIRONMENT,
        design="historical-trained fixed-model UMi-corrected geometry replay",
        actual_simulations=EVALUATIONS, logical_successful_observations=EVALUATIONS,
        evaluation_observations=EVALUATIONS, independent_scenarios=INPUTS,
        repeated_previously_evaluated_input_scenarios=INPUTS, newly_unseen_input_scenarios=0,
        complete_fixed_models=11, learned_models=9, actual_deterministic_rules=2,
        training_simulations=0, training_simulations_excluded_from_endpoints=0,
        source_evaluation_training_updates=0, analysis_simulations=0, analysis_training_updates=0,
        source_training_environment="historical original training geometry; unchanged fixed cp11 models",
        old_evaluation_results_pooled=0, training_results_pooled=0,
        authorized_retry_simulations=0, offline_recovery_new_simulations=0,
        historical_failures_counted_as_this_batch_dispatches=0,
        isolated_simulator=str(BINARY), input_plan_source_reference=str(SOURCE_PLAN),
        method_success_claim=False, deployment_SLA_guarantee_claim=False,
        interpretation=interpretation(), B_started=False, C_started=False,
    )
    return result


def save_exclusive(path, value):
    # Complete serialization precedes file creation, matching the repaired
    # persistence behavior; failed serialization cannot publish a partial JSON.
    text = json.dumps(value, ensure_ascii=True, allow_nan=False, indent=2) + "\n"
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)


def run_analysis(run_dir, tasks=None):
    run = Path(run_dir).resolve()
    if run.drive.upper() != "D:" or run.parent != HERE:
        raise ValueError(f"Outputs must remain directly below {HERE} on D:")
    if tasks is None:
        task_path = run / "tasks.json"
        if not task_path.exists():
            task_path = HERE / "plan.json"
        tasks = read_json(task_path)
    if isinstance(tasks, dict):
        tasks = tasks.get("tasks", tasks.get("jobs"))
    evaluation = load_evaluation(run, tasks)
    models = legacy.model_rows(evaluation)
    stats = legacy.statistics_for_models(models)
    stats.update(environment=ENVIRONMENT, repeated_previously_evaluated_input_scenarios=INPUTS,
                 newly_unseen_input_scenarios=0, old_evaluation_results_pooled=0)
    compact = concise_summary(stats)
    output, validation = run / "analysis", run / "validation"
    targets = [output / "analysis_summary.json", output / "data_and_disclosure.json"] + [
        validation / row["model_key"] / "independent_summary.json" for row in models]
    if any(path.exists() for path in targets):
        raise FileExistsError("After-only analysis never overwrites existing evidence")
    output.mkdir(exist_ok=False)
    validation.mkdir(exist_ok=True)
    for row in models:
        row.update(environment=ENVIRONMENT, input_reuse="same previously evaluated100 inputs; corrected geometry",
                   training_environment="historical training geometry", source_results_pooled=0)
        folder = validation / row["model_key"]
        folder.mkdir(exist_ok=False)
        row["summary_reference"] = str(folder / "independent_summary.json")
        save_exclusive(folder / "independent_summary.json", row)
    compact.update(analysis_summary_reference=str(targets[0]), data_and_disclosure_reference=str(targets[1]))
    save_exclusive(targets[1], {
        "scope": "Only this authorized1100 fixed-model corrected-geometry replay matrix; no old outcomes",
        "environment": ENVIRONMENT, "design": compact["design"],
        "model_summaries_all_terminal_metrics_and_source_references": models,
        "independent_statistics": stats,
        "evaluation_jobs": [row["job"] for row in sorted(evaluation, key=lambda row: row["job"]["id"])],
        "actual_simulator_dispatches": EVALUATIONS, "logical_successful_observations": EVALUATIONS,
        "logical_evaluation_observations": EVALUATIONS, "training_simulations":0,
        "optimizer_updates":0, "authorized_retries":0, "offline_recovery_simulations":0,
        "old_evaluation_results_pooled":0, "training_results_pooled":0,
        "repeated_previously_evaluated_input_scenarios":INPUTS, "newly_unseen_input_scenarios":0,
        "interpretation": interpretation(), "plan_reference":str(HERE / "plan.json"),
        "source_plan_reference":str(SOURCE_PLAN), "isolated_simulator":str(BINARY),
    })
    save_exclusive(targets[0], compact)
    return compact


build_analysis = run_analysis


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", default=str(HERE / "run_001"))
    parser.add_argument("--tasks")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.execute:
        result = run_analysis(args.run_dir, read_json(args.tasks) if args.tasks else None)
    else:
        result = {"status":"read_only_inventory_no_analysis", "files_written":0, "simulations":0,
                  "training":0, "updates":0, "retries":0, "required_evaluation_results":EVALUATIONS,
                  "actual_result_files":len(list((Path(args.run_dir) / "episodes").glob("*/result.json")))}
    print(json.dumps(result, ensure_ascii=True, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
