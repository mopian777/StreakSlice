"""One explicitly authorized UMi-domain fixed-checkpoint reassessment.

This is an evaluation-only plan. The original 100 inputs and all nine cp11
checkpoints are read-only references; no training, choice, retry or recovery.
"""
from __future__ import annotations
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
FAIR = HERE.parent
OLD_BASE = FAIR / 'cp11_main_comparison_20261017'
WORK = Path('D:/1/Codex/Codex/2026-08-20/w/work/burstshield_ns3/randomized_20260927')
RUN = HERE / 'run_001'
SIM_EVAL = HERE / 'build_001/deadline_sim'
CP11 = FAIR / 'actor_adam_reset_pilot_20261015/run_001/models'
SEEDS = (90101, 90202, 90303)
GROUPS = ('ru64', 'original_tail_q', 'dirppo_matched', 'guarded_equal', 'guarded_maxweight')
TOTAL, TRAIN, EVAL = 1100, 0, 1100
MIN_FREE, CONCURRENCY, BATCH = 20 * 2**30, 10, 25
ENVIRONMENT = 'umi_compliant_geometry_20261019'
COMPLETED = 'completed_1100_fixed_models_umi_compliant_reassessment'


def read(path):
    with Path(path).open(encoding='utf-8') as stream:
        return json.load(stream, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def encoder(value):
    if isinstance(value, Path): return str(value)
    if hasattr(value, 'tolist'): return value.tolist()
    if hasattr(value, 'item'): return value.item()
    raise TypeError(type(value).__name__)


def save(path, value):
    path = Path(path)
    if not path.resolve().is_relative_to(HERE.resolve()):
        raise ValueError('New output must remain within the isolated UMi reassessment')
    # Complete encoding before exclusive creation, preserving shape and old evidence.
    encoded = json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False, default=encoder) + '\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(encoded); stream.flush()


def jobs():
    source = read(OLD_BASE / 'plan.json')['jobs']
    source = [row for row in source if row['phase'] == 'independent_test']
    if len(source) != TOTAL or [j['id'] for j in source] != list(range(192, 1292)):
        raise ValueError('Original source plan must contain exactly the original 1100 evaluations')
    return [dict(row, id=i, source_id=row['id'], environment=ENVIRONMENT)
            for i, row in enumerate(source)]


def folder(job):
    return RUN / 'episodes' / f"j{job['id']:04d}"


def fixed_model(job):
    if job.get('phase') != 'independent_test': raise ValueError('No training is authorized')
    if job['arm'] == 'dirppo_matched':
        return OLD_BASE / 'run_001/models/dirppo_matched' / job['model_id'] / 'checkpoint_11.pt'
    if job['arm'] in ('ru64', 'original_tail_q'):
        arm = 'trajectory_ru' if job['arm'] == 'ru64' else 'original_tail_q'
        return CP11 / arm / job['model_id'] / 'checkpoint_11.pt'
    if job['arm'] not in GROUPS[-2:]: raise ValueError('Unknown fixed configuration')
    return None


def model_inventory():
    paths = sorted({fixed_model(j) for j in jobs() if fixed_model(j) is not None})
    if len(paths) != 9: raise ValueError('Exactly nine existing cp11 files required')
    return {str(p): dict(bytes=p.stat().st_size, mtime_ns=p.stat().st_mtime_ns) for p in paths}


def verify_models(expected):
    if model_inventory() != expected: raise RuntimeError('A referenced fixed cp11 changed')


def check_configuration(require_simulator=True):
    rows = jobs()
    if HERE.drive.upper() != 'D:' or [j['id'] for j in rows] != list(range(TOTAL)):
        raise ValueError('Exactly 1100 new local evaluation IDs on D are authorized')
    expected = [
        ('ru64', 0), ('ru64', 1), ('ru64', 2),
        ('original_tail_q', 0), ('original_tail_q', 1), ('original_tail_q', 2),
        ('dirppo_matched', 0), ('dirppo_matched', 1), ('dirppo_matched', 2),
        ('guarded_equal', None), ('guarded_maxweight', None),
    ]
    for i in range(100):
        batch = rows[i * 11:(i + 1) * 11]
        if [(j['arm'], j['init_seed_index']) for j in batch] != expected:
            raise ValueError('Original eleven-way paired layout changed')
        for j in batch:
            if (j['phase'] != 'independent_test' or j['input_index'] != i
                    or j['source_id'] != j['id'] + 192 or j['scenario_seed'] != 823200 + i
                    or j['sampling_seed'] != 1962000 + i or j['simulation_seconds'] != 40
                    or j['bandwidth_hz'] != 2500000
                    or j['family'] != ('burst' if i % 2 == 0 else 'compound')
                    or j['environment'] != ENVIRONMENT):
                raise ValueError('Only the original input/model/seed/config pairing may be evaluated')
    model_inventory()
    if require_simulator and not SIM_EVAL.is_file(): raise FileNotFoundError(SIM_EVAL)
    return rows


def plan():
    rows = check_configuration(require_simulator=False)
    return dict(study='UMi-domain fixed-cp11 reassessment of the original 100 inputs',
        jobs=rows, maximum_dispatches=TOTAL, evaluation_dispatches=EVAL,
        training_dispatches=0, new_updates=0, actual_new_epoch_rows=0,
        maximum_concurrency=CONCURRENCY, batch_size=BATCH, archive_batches=44,
        evaluation_unique_inputs=100, initialization_seeds=list(SEEDS), groups=list(GROUPS),
        source_plan=str(OLD_BASE / 'plan.json'), evaluation_simulator=str(SIM_EVAL),
        source_id_range=[192, 1291], local_id_range=[0, 1099],
        geometry=dict(gNB_height_m=10.0, UE_height_m=1.5,
                      original_horizontal_sampling_rectangle_m=[3.0, 3.0], UE_relative_x_offset_m=20.0,
                      horizontal_distance_derived_range_m=[20.0, (23.0**2 + 3.0**2)**0.5]),
        interpretation='Same-input UMi-domain transfer reassessment of historically trained fixed models; not new unseen inputs and not end-to-end compliant-domain training',
        source_models_read_only=True, source_results_read_only=True,
        automatic_retry=False, automatic_resume=False, replacement_seeds=False,
        model_selection=False, extra_tasks=False, minimum_free_space_GiB=20,
        archive_policy='At most 25 successful actually exited workers per batch; all workers drain before serial member-byte-readback archival and analysis; only verified current-batch plain equivalents removed',
        disk_reservation='Ten shared slots, max(256MiB, 2*maximum observed completed episode directory) per occupied/proposed slot; worker and Episode 20GiB checks retained; estimate is not a growth bound',
        analysis=dict(primary='Per-model 100-input H-CVaR95 then three fixed models equally averaged; rules have one vector',
            difference='RU64 minus each comparator, negative favorable', secondary='mean H',
            bootstrap_replicates=2000, bootstrap_seed=88581,
            strata=dict(burst=50, compound=50), paired_same_indices_across_models=11,
            fixed_initializations_only=True, simultaneous_multi_metric_guarantee=False),
        runtime_100ms_required=False, inference_risk_logit_correction=False,
        B_started=False, C_started=False)
