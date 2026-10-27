"""Reuse the existing verified-member archiver in this new scope only.

The two old rule-retry IDs have no meaning here. Local IDs 201 and 202 use
ordinary new episode and receipt paths, with no old recovery/retry manifest.
"""
from __future__ import annotations
import importlib.util
import json
from pathlib import Path
from study_umi import HERE, OLD_BASE

_spec = importlib.util.spec_from_file_location('_existing_storage_for_umi', OLD_BASE / 'storage1292.py')
existing = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(existing)
existing.APPROVED_ROOT = HERE


def _job_folders(run, phase, jobs):
    if phase != 'eval' or not jobs or len(jobs) > existing.MAX_BATCH:
        raise ValueError('Only nonempty evaluation batches of at most 25 are authorized')
    prior_sources = set()
    for path in (run / 'archives').glob('*/archive_index.json'):
        existing._contained(path, run)
        prior_sources.update(json.loads(path.read_text(encoding='utf-8')).get('episode_relative_paths', []))
    folders = []
    for job in jobs:
        i = job.get('id')
        if type(i) is not int or not 0 <= i < 1100 or job.get('phase') != 'independent_test':
            raise ValueError('Only the 1100 local evaluation IDs are authorized')
        folder = existing._contained(run / 'episodes' / f'j{i:04d}', run)
        if folder.relative_to(run).as_posix() in prior_sources:
            raise ValueError('No repeat archival is authorized')
        result = json.loads((folder / 'result.json').read_text(encoding='utf-8'))
        if result.get('job') != job or not isinstance(result.get('result'), dict) or not result['result']:
            raise ValueError('Complete local evaluation identity/result required')
        native = json.loads((folder / 'native_exit.json').read_text(encoding='utf-8'))
        processes = native.get('processes', [])
        if (native.get('all_exited') is not True or native.get('worker_exit_code') != 0
                or native.get('simulation_dispatches') != 1 or len(processes) != 1
                or processes[0].get('actual_exit_code') != 0 or (folder / 'failure.json').exists()):
            raise ValueError('Successful actual native client exit is required')
        receipt = existing._contained(run / 'exits' / f'worker_{i:04d}.json', run)
        exit_info = json.loads(receipt.read_text(encoding='utf-8'))
        if exit_info.get('actual_exit_code') != 0 or exit_info.get('id') != i or exit_info.get('job') != job:
            raise ValueError('Matching actual Windows worker exit0 receipt required')
        index = json.loads((folder / 'evaluation_index.json').read_text(encoding='utf-8'))
        if index.get('kind') != 'evaluation_index': raise ValueError('Canonical evaluation index required')
        existing._contained(folder / index['canonical_controller'], folder)
        existing._tree(folder, run)
        folders.append(folder)
    if len(folders) != len(set(folders)): raise ValueError('Duplicate local archive job')
    return sorted(folders)


existing._job_folders = _job_folders
write_eval = existing.write_eval
open_evidence = existing.open_evidence
archive_batch = existing.archive_batch
