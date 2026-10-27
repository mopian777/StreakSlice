"""Owned actual-child-exit wrapper; one new fixed 1100-evaluation run only."""
import os
import subprocess
import sys
import time
import traceback
from study_umi import *
from run_umi import add_paths, wait_owned


def main():
    add_paths(); from progress_store import ProgressStore
    chain = HERE / 'chain_run_001'; chain.mkdir(exist_ok=False); progress = ProgressStore(chain)
    save(chain / 'start.json', dict(pid=os.getpid(), started_unix_s=time.time(), maximum_dispatches=TOTAL,
        maximum_concurrency=CONCURRENCY, new_training=0, new_updates=0, new_retries=0, automatic_retry=False))
    child = None; code = None; failure = None; observations = []
    log = (chain / 'driver.log').open('x', encoding='utf-8')
    try:
        child = subprocess.Popen([sys.executable, '-B', str(HERE / 'run_umi.py'), '--execute'],
            cwd=WORK, stdout=log, stderr=subprocess.STDOUT)
        save(chain / 'stage_start.json', dict(child_pid=child.pid, started_unix_s=time.time(), stage='fixed1100_umi'))
        progress.write(dict(stage='fixed1100_umi', child_pid=child.pid, maximum_dispatches=TOTAL, maximum_concurrency=CONCURRENCY))
        code, observations = wait_owned(child)
    except BaseException as error:
        failure = dict(type=type(error).__name__, message=str(error), traceback=traceback.format_exc())
    finally:
        if child is not None and code is None: code, observations = wait_owned(child)
        log.close()
    save(chain / 'stage_exit.json', dict(actual_child_exit_code=code, child_pid=None if child is None else child.pid,
        failure=failure, wait_observation_errors=observations, finished_unix_s=time.time()))
    summary = read(RUN / 'summary.json') if (RUN / 'summary.json').is_file() else {}
    ok = (code == 0 and failure is None and not observations and summary.get('status') == COMPLETED
        and summary.get('successful') == TOTAL and summary.get('dispatched') == TOTAL
        and summary.get('failed') == 0 and summary.get('unknown') == 0 and summary.get('archived_batches') == 44
        and summary.get('all_spawned_workers_drained') is True and summary.get('fixed_models_unchanged') is True
        and summary.get('new_updates') == 0 and summary.get('retries') == 0)
    result = dict(status='completed_fixed1100_umi_sequence' if ok else 'failed_stopped_no_retry',
        actual_child_exit_code=code, failure=failure, wait_observation_errors=observations,
        maximum_dispatches=TOTAL, maximum_concurrency=CONCURRENCY,
        automatic_retry=False, automatic_resume=False, B_started=False, C_started=False)
    save(chain / 'summary.json', result); progress.write(result)
    return 0 if ok else 1


if __name__ == '__main__': raise SystemExit(main())
