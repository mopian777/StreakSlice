"""1100 fixed-model evaluations, ten slots, serial verified archival; no retry."""
from __future__ import annotations
import argparse
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
from study_umi import *


def add_paths():
    for path in (OLD_BASE, FAIR, FAIR / 'independent_r10_test_20261014',
                 FAIR.parent / 'simulator_cp11_compatibility_20261016',
                 FAIR.parent / 'simulator_cohort_regression_20261016'):
        if str(path) not in sys.path: sys.path.append(str(path))


def worker_module():
    """Bind the existing worker implementation to new output and model routes.

    This mutates only this child interpreter's module globals, not any source.
    storage1292 is explicitly installed as the local archive/index adapter.
    """
    add_paths()
    import storage_umi
    sys.modules['storage1292'] = storage_umi.existing
    original = importlib.import_module('run1292')
    bindings = dict(HERE=HERE, FAIR=FAIR, RUN=RUN, WORK=WORK, TOTAL=TOTAL,
                    TRAIN=0, EVAL=EVAL, CONCURRENCY=CONCURRENCY, BATCH=BATCH,
                    MIN_FREE=MIN_FREE, SIM_EVAL=SIM_EVAL, SIM_TRAIN=SIM_EVAL,
                    jobs=jobs, folder=folder, fixed_model=fixed_model,
                    read=read, save=save, encoder=encoder)
    for key, value in bindings.items(): setattr(original, key, value)
    return original


def worker(index):
    if type(index) is not int or not 0 <= index < TOTAL:
        raise ValueError('Only one of the 1100 new local evaluation IDs is authorized')
    return worker_module().worker(index)


def wait_owned(child):
    """Wait for the actual Popen child even after an observation interruption."""
    errors = []
    while True:
        try: return child.wait(), errors
        except BaseException as error:
            errors.append(dict(type=type(error).__name__, message=str(error)))
            time.sleep(.2)


def directory_bytes(path):
    return sum(p.stat().st_size for p in path.rglob('*') if p.is_file())


class Dispatcher:
    def __init__(self, publish, popen=subprocess.Popen):
        self.publish, self.popen = publish, popen
        self.active = {}; self.started = set(); self.successful = set()
        self.failed = set(); self.unknown = set(); self.max_completed_bytes = 0

    def observe_exit(self, i, code):
        child, log, job = self.active[i]
        actual, errors = wait_owned(child)
        log.close()
        # Own the receipt before removing this child from our drain inventory.
        if actual != code: raise RuntimeError('Actual wait and poll exits disagree')
        save(RUN / 'exits' / f'worker_{i:04d}.json',
             dict(id=i, pid=child.pid, actual_exit_code=actual, job=job,
                  finished_unix_s=time.time(), wait_observation_errors=errors))
        del self.active[i]
        if errors:
            self.unknown.add(i); raise RuntimeError('Worker wait observation error; no further dispatch')
        valid, detail = False, None
        try:
            result = read(folder(job) / 'result.json'); native = read(folder(job) / 'native_exit.json')
            valid = (actual == 0 and result.get('job') == job
                and not (folder(job) / 'failure.json').exists()
                and result.get('training_updates_during_collection') == 0
                and result.get('policy_parameters_unchanged') is True
                and native.get('worker_exit_code') == 0 and native.get('all_exited') is True
                and native.get('simulation_dispatches') == 1 and len(native.get('processes', [])) == 1
                and native['processes'][0].get('actual_exit_code') == 0)
        except BaseException as error: detail = repr(error)
        if not valid:
            (self.unknown if detail is not None and actual == 0 else self.failed).add(i)
            if detail: save(RUN / 'exits' / f'worker_{i:04d}_result_error.json', dict(error=detail))
        else:
            self.successful.add(i)
            self.max_completed_bytes = max(self.max_completed_bytes, directory_bytes(folder(job)))

    def finish(self, i):
        code = self.active[i][0].poll()
        if code is not None: self.observe_exit(i, code)

    def drain(self):
        errors = []
        for i in list(self.active):
            child, log, job = self.active[i]
            try:
                code, observation_errors = wait_owned(child)
                if observation_errors: self.unknown.add(i)
                self.observe_exit(i, code)
                if observation_errors: raise RuntimeError(str(observation_errors))
            except BaseException as error:
                errors.append(repr(error)); self.unknown.add(i)
                # observe_exit may have failed writing its receipt. Still wait the
                # actual child and preserve a separate error receipt, never retry.
                if i in self.active:
                    code, observation_errors = wait_owned(child)
                    try: log.close()
                    except BaseException: pass
                    del self.active[i]
                    try:
                        save(RUN / 'exits' / f'worker_{i:04d}_drain_error.json',
                             dict(id=i, job=job, pid=child.pid, actual_exit_code=code,
                                  error=repr(error), wait_observation_errors=observation_errors))
                    except BaseException as receipt_error: errors.append(repr(receipt_error))
        if errors: raise RuntimeError('Actual drain/receipt errors: ' + '; '.join(errors))

    def run_batch(self, batch):
        queue = list(batch); failure = None
        try:
            while queue or self.active:
                for i in list(self.active): self.finish(i)
                if self.failed or self.unknown:
                    raise RuntimeError('Failure or UNKNOWN: stop dispatch and naturally drain all siblings')
                while queue and len(self.active) < CONCURRENCY:
                    job = queue[0]; i = job['id']
                    if i in self.started or len(self.started) >= TOTAL:
                        raise RuntimeError('No duplicate, replacement or extra dispatch is authorized')
                    paths = (folder(job), RUN / 'starts' / f'worker_{i:04d}.json',
                             RUN / 'exits' / f'worker_{i:04d}.json', RUN / 'logs' / f'worker_{i:04d}.log')
                    if any(p.exists() for p in paths): raise ValueError('Prior local evidence exists; never retry or overwrite')
                    reserve = max(256 * 2**20, 2 * self.max_completed_bytes)
                    if shutil.disk_usage(HERE).free < MIN_FREE + reserve * (len(self.active) + 1):
                        raise RuntimeError('Shared ten-slot 20GiB plus in-flight reservation protection')
                    log = paths[-1].open('x', encoding='utf-8')
                    try:
                        child = self.popen([sys.executable, '-B', str(HERE / 'run_umi.py'), '--worker', str(i)],
                            cwd=WORK, stdout=log, stderr=subprocess.STDOUT)
                    except BaseException: log.close(); raise
                    # Record ownership immediately after Popen; any publication
                    # failure now stops new dispatch but cannot orphan this child.
                    self.active[i] = (child, log, job); self.started.add(i); queue.pop(0)
                    save(paths[1], dict(id=i, job=job, pid=child.pid, started_unix_s=time.time()))
                    self.publish()
                self.publish()
                if self.active: time.sleep(1)
        except BaseException as error: failure = error
        finally:
            try: self.drain()
            except BaseException as error:
                if failure is None: failure = error
        if failure is not None: raise failure
        if any(j['id'] not in self.successful for j in batch): raise RuntimeError('Incomplete batch')


def archive(number):
    rows = jobs(); batch = rows[number * BATCH:(number + 1) * BATCH]
    if not 0 <= number < 44 or not batch or not (RUN / 'start.json').is_file():
        raise ValueError('Only one explicit current-run batch is authorized')
    from storage_umi import archive_batch
    return archive_batch(RUN, 'eval', f'{number:03d}', batch, workers_drained=True, active_workers=0)


def execute():
    add_paths()
    from progress_store import ProgressStore
    rows = check_configuration()
    test = read(HERE / 'unit_tests/summary.json')
    if test.get('successful') is not True or test.get('simulations_started') != 0:
        raise ValueError('Ordinary no-simulation regressions have not passed')
    if read(HERE / 'plan.json') != plan(): raise ValueError('Saved local plan and implementation differ')
    # No reuse of a partial run and no implicit recovery or resumption.
    RUN.mkdir(exist_ok=False)
    for name in ('episodes', 'logs', 'starts', 'exits', 'stage_exits', 'validation'): (RUN / name).mkdir()
    models = model_inventory(); save(RUN / 'fixed_models_before.json', models)
    save(RUN / 'start.json', dict(pid=os.getpid(), started_unix_s=time.time(), maximum_dispatches=TOTAL,
        maximum_concurrency=CONCURRENCY, evaluation=EVAL, training=0, updates=0, retries=0,
        automatic_retry=False, automatic_resume=False, source_plan=str(OLD_BASE / 'plan.json'),
        evaluation_simulator=str(SIM_EVAL), free_bytes=shutil.disk_usage(HERE).free))
    progress = ProgressStore(RUN); stage = 'preparing'; archives = []; active_stage = None
    dispatcher = None
    def publish():
        success = set() if dispatcher is None else dispatcher.successful
        started = set() if dispatcher is None else dispatcher.started
        progress.write(dict(stage=stage, pid=os.getpid(), successful=len(success), logical_successful=len(success),
            dispatched=len(started), actual_dispatched=len(started), planned=TOTAL,
            failed=0 if dispatcher is None else len(dispatcher.failed), unknown=0 if dispatcher is None else len(dispatcher.unknown),
            train_completed=0, evaluation_completed=len(success), new_updates=0, actual_epoch_rows=0, retries=0,
            complete_paired_inputs=sum(all(i * 11 + k in success for k in range(11)) for i in range(100)),
            complete_evaluation_models=sum(all(i * 11 + k in success for i in range(100)) for k in range(11)),
            active_jobs=[] if dispatcher is None else [dict(id=i, source_id=j['source_id'], pid=p.pid,
                arm=j['arm'], phase=j['phase'], scenario_seed=j['scenario_seed']) for i, (p, _, j) in dispatcher.active.items()],
            active_stage=active_stage, archived_batches=len(archives), archive_batches_planned=44,
            maximum_concurrency=CONCURRENCY, free_bytes=shutil.disk_usage(HERE).free,
            maximum_observed_completed_directory_bytes=0 if dispatcher is None else dispatcher.max_completed_bytes,
            automatic_retry=False, automatic_resume=False, model_selection=False, B_started=False, C_started=False))
    dispatcher = Dispatcher(publish)
    def invoke(arguments, key):
        nonlocal active_stage
        if dispatcher.active: raise RuntimeError('Serial archival/analysis cannot overlap active workers')
        child = None; code = None; failure = None; observations = []
        log = (RUN / 'logs' / f'{key}.log').open('x', encoding='utf-8')
        try:
            child = subprocess.Popen([sys.executable, '-B', str(HERE / 'run_umi.py'), *arguments],
                cwd=WORK, stdout=log, stderr=subprocess.STDOUT)
            active_stage = dict(key=key, pid=child.pid)
            save(RUN / 'stage_exits' / f'{key}_start.json', dict(pid=child.pid, started_unix_s=time.time()))
            publish(); code, observations = wait_owned(child)
        except BaseException as error: failure = error
        finally:
            if child is not None and code is None: code, observations = wait_owned(child)
            log.close(); active_stage = None
            save(RUN / 'stage_exits' / f'{key}_exit.json', dict(actual_exit_code=code,
                pid=None if child is None else child.pid, finished_unix_s=time.time(), wait_observation_errors=observations))
        if failure is not None: raise failure
        if code != 0 or observations: raise RuntimeError(f'{key} actualexit={code}, observation errors={observations}; no retry')
        publish()
    failure = None
    try:
        publish()
        for number in range(44):
            batch = rows[number * BATCH:(number + 1) * BATCH]
            stage = 'umi_fixed_model_evaluation'; dispatcher.run_batch(batch)
            stage = 'verified_serial_archival'; invoke(['--archive', str(number)], f'archive_eval_{number:03d}')
            index = read(RUN / 'archives' / f'eval_{number:03d}' / 'archive_index.json')
            if (index.get('status') != 'completed_verified_archival' or index.get('job_ids') != [j['id'] for j in batch]
                    or any(e.get('verified') is not True or e.get('source_removed') is not True for e in index['entries'])):
                raise RuntimeError('Archive child exited but full batch verified-removal status incomplete')
            archives.append(number); verify_models(models); publish()
        save(RUN / 'eval_stage_archived.json', dict(status='all_batches_verified_archived', successful=TOTAL,
             batches=archives, new_training=0, new_updates=0, new_retries=0))
        stage = 'analyzing'; invoke(['--analyze'], 'analysis'); verify_models(models)
        analysis = read(RUN / 'analysis/analysis_summary.json')
        disclosure = read(RUN / 'analysis/data_and_disclosure.json')
        validation = list((RUN / 'validation').glob('*/independent_summary.json'))
        if (analysis.get('status') != 'completed_umi_compliant_geometry_fixed1100_replay_analysis'
                or analysis.get('actual_simulations') != TOTAL or len(validation) != 11
                or disclosure.get('actual_simulator_dispatches') != TOTAL
                or analysis.get('training_simulations') != 0 or analysis.get('authorized_retry_simulations') != 0):
            raise RuntimeError('Completed new-only analysis and eleven fixed-policy summaries required')
        if (dispatcher.successful != set(range(TOTAL)) or dispatcher.started != set(range(TOTAL))
                or dispatcher.failed or dispatcher.unknown or len(archives) != 44):
            raise RuntimeError('Final finite evaluation counts incomplete')
        stage = COMPLETED
    except BaseException as error:
        failure = dict(type=type(error).__name__, message=str(error), traceback=traceback.format_exc())
        stage = 'failed_stopped_no_retry'
    finally:
        try: dispatcher.drain()
        except BaseException as error:
            if failure is None: failure = dict(type=type(error).__name__, message=str(error), traceback=traceback.format_exc())
            stage = 'failed_stopped_no_retry'
        summary = dict(status=stage, successful=len(dispatcher.successful), dispatched=len(dispatcher.started),
            failed=len(dispatcher.failed), unknown=len(dispatcher.unknown), evaluation_completed=len(dispatcher.successful),
            train_completed=0, new_updates=0, actual_epoch_rows=0, retries=0, archived_batches=len(archives),
            all_spawned_workers_drained=not dispatcher.active, failure=failure,
            fixed_models_unchanged=model_inventory() == models, free_bytes=shutil.disk_usage(HERE).free,
            automatic_retry=False, automatic_resume=False, model_selection=False, B_started=False, C_started=False)
        save(RUN / 'stage_exit.json', summary); save(RUN / 'summary.json', summary); publish()
    return 0 if failure is None else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--prepare-plan', action='store_true'); modes.add_argument('--execute', action='store_true')
    modes.add_argument('--worker', type=int); modes.add_argument('--archive', type=int)
    modes.add_argument('--analyze', action='store_true')
    args = parser.parse_args(); add_paths()
    if args.prepare_plan: save(HERE / 'plan.json', plan()); return 0
    if args.execute: return execute()
    if args.worker is not None: return worker(args.worker)
    if args.archive is not None: archive(args.archive); return 0
    if args.analyze:
        from analyze_umi import run_analysis
        run_analysis(RUN, jobs()); return 0
    return 1


if __name__ == '__main__': raise SystemExit(main())
